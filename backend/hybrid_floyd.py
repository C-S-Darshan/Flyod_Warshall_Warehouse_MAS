import numpy as np
import heapq
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Tuple, Set, Any

# --- Pydantic Models for Request/Response ---

class AgentConfig(BaseModel):
    id: str
    start: Tuple[int, int]
    goal: Tuple[int, int]

class SimulationConfig(BaseModel):
    rows: int = 10
    cols: int = 10
    shelves: List[Tuple[int, int]] = [
        (4, 4), (4, 5), (5, 4), (5, 5),
        (2, 2), (2, 3), (2, 4),
        (7, 6), (7, 7), (7, 8)
    ]
    agents: List[AgentConfig] = [
        {"id": "A", "start": (0, 0), "goal": (9, 9)},
        {"id": "B", "start": (0, 9), "goal": (9, 0)},
        {"id": "C", "start": (9, 0), "goal": (0, 9)},
        {"id": "D", "start": (9, 9), "goal": (0, 0)},
        {"id": "E", "start": (5, 0), "goal": (5, 9)},
    ]

class AgentResult(BaseModel):
    id: str
    path: List[Tuple[int, int]] # List of (row, col) coordinates
    rerouted: bool
    reroute_time: int = 0
    original_segment: List[Tuple[int, int]] = []
    dijkstra_segment: List[Tuple[int, int]] = []

class SimulationResponse(BaseModel):
    agents_data: List[AgentResult]
    conflict_map: Dict[int, List[Tuple[int, int]]] # {time: [(row, col), ...]}
    conflict_logs: List[str]
    max_time_for_animation: int
    grid_size: Tuple[int, int] # (rows, cols)
    shelves: List[Tuple[int, int]]

# --- Core Simulation Logic (Adapted from previous Python script) ---

# Global variables for grid dimensions and adjacency matrix,
# will be set by build_graph for the current simulation run.
_rows: int = 0
_cols: int = 0
_adj_matrix: np.ndarray = None
# Global variable to store shelves as a set for faster lookup in dijkstra
_shelves_set: Set[Tuple[int, int]] = set()

def in_bounds(x: int, y: int) -> bool:
    """Checks if a given coordinate is within the grid boundaries."""
    return 0 <= x < _rows and 0 <= y < _cols

def node_id(x: int, y: int) -> int:
    """Converts (x, y) coordinates to a unique node ID."""
    return x * _cols + y

def id_to_coord(id_: int) -> Tuple[int, int]:
    """Converts a node ID back to (x, y) coordinates."""
    return divmod(id_, _cols)

def build_graph(rows: int, cols: int, shelves: Set[Tuple[int, int]]) -> np.ndarray:
    """
    Builds the adjacency matrix for the warehouse graph.
    Shelves are considered obstacles (no edges through them).
    """
    global _rows, _cols, _adj_matrix # Declare global to modify
    _rows = rows
    _cols = cols
    N = rows * cols
    _adj_matrix = np.full((N, N), np.inf) # Initialize with infinity for no connection
    
    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Directions: Up, Down, Left, Right

    for x in range(rows):
        for y in range(cols):
            if (x, y) in shelves:
                continue # Cannot move through shelves
            a = node_id(x, y)
            _adj_matrix[a][a] = 0 # Distance to self is 0

            for dx, dy in DIRS:
                nx, ny = x + dx, y + dy
                if in_bounds(nx, ny) and (nx, ny) not in shelves:
                    b = node_id(nx, ny)
                    _adj_matrix[a][b] = 1 # Unit cost for moving to adjacent node
    return _adj_matrix

def floyd_with_path(adj: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Implements the Floyd-Warshall algorithm to find all-pairs shortest paths
    and reconstruct paths.
    Returns the distance matrix and the path reconstruction matrix.
    """
    N = adj.shape[0]
    dist = adj.copy()
    path = np.full((N, N), -1) # Stores the intermediate node in the shortest path

    # Initialize path matrix for direct connections
    for i in range(N):
        for j in range(N):
            if adj[i][j] == 1: # If directly connected
                path[i][j] = j # Next hop is the destination itself

    for k in range(N): # Intermediate node
        for i in range(N): # Start node
            for j in range(N): # End node
                if dist[i][k] != np.inf and dist[k][j] != np.inf: # Avoid infinity arithmetic
                    if dist[i][k] + dist[k][j] < dist[i][j]:
                        dist[i][j] = dist[i][k] + dist[k][j]
                        path[i][j] = path[i][k] # Update next hop from i to j via k

    return dist, path

def reconstruct_path(i: int, j: int, path_matrix: np.ndarray) -> List[int]:
    """
    Reconstructs the shortest path from node i to node j using the path_matrix.
    """
    if i == j:
        return [i]
    if path_matrix[i][j] == -1: # No path found by Floyd-Warshall
        return []

    path = [i]
    current = i
    while current != j:
        next_node = path_matrix[current][j]
        if next_node == -1: # Should not happen if path_matrix[i][j] was not -1 initially
            return [] # Indicates an issue in path reconstruction or matrix
        path.append(next_node)
        current = next_node
    return path

def path_to_coords(path_ids: List[int]) -> List[Tuple[int, int]]:
    """Converts a list of node IDs to a list of (x, y) coordinates."""
    return [id_to_coord(n) for n in path_ids]

def dijkstra_with_avoidance(start_id: int, goal_id: int, occupied_at_time: Dict[int, Set[int]], current_max_time: int) -> List[int]:
    """
    Implements Dijkstra's algorithm to find a path from start to goal,
    avoiding nodes occupied by other agents at specific time steps.
    `occupied_at_time`: A dictionary where keys are time steps and values are sets of occupied node IDs.
    `current_max_time`: The current maximum time for simulation, used to limit Dijkstra's search.
    """
    visited: Set[Tuple[int, int]] = set() # Stores (node_id, time_step) to avoid cycles and re-exploring
    pq: List[Tuple[int, int, List[int]]] = [(0, start_id, [start_id])] # (cost, current_node_id, path_list)

    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Directions: Up, Down, Left, Right
    
    while pq:
        cost, current_id, path = heapq.heappop(pq)

        if current_id == goal_id:
            return path # Found shortest path

        # Check if this (node, time) state has already been visited with a shorter or equal path
        if (current_id, cost) in visited:
            continue
        visited.add((current_id, cost))

        # Iterate over possible neighbors using DIRS and check against _adj_matrix
        for dx, dy in DIRS:
            nx, ny = id_to_coord(current_id) # Get current node's (x,y)
            nx, ny = nx + dx, ny + dy # Calculate neighbor's (x,y)

            if in_bounds(nx, ny) and (nx, ny) not in _shelves_set: # Check against shelves
                neighbor_id = node_id(nx, ny)
                
                # Verify that there's an actual edge in the graph
                if _adj_matrix[current_id][neighbor_id] == 1: # Assuming unit cost for edges
                    next_time = cost + 1 # Each step takes 1 unit of time

                    # Limit Dijkstra's search to prevent excessively long paths and infinite loops
                    if next_time >= current_max_time * 3: # Allow more flexibility for rerouting
                        continue

                    # Check for time window conflict: is the neighbor occupied at the next time step?
                    if neighbor_id in occupied_at_time.get(next_time, set()):
                        continue # Skip this path if the next node is occupied

                    heapq.heappush(pq, (next_time, neighbor_id, path + [neighbor_id]))
    return [start_id] # Return start node if no path found

def run_simulation(config: SimulationConfig) -> Tuple[List[AgentResult], Dict[int, List[Tuple[int, int]]], List[str], int]:
    """
    Simulates agent movement, detects conflicts, and recalculates paths.
    """
    global _shelves_set # Declare global to modify
    _shelves_set = set(tuple(s) for s in config.shelves) # Convert list of lists to set of tuples

    adj = build_graph(config.rows, config.cols, _shelves_set)
    dist, path_matrix = floyd_with_path(adj)
    
    time_window: Dict[int, Dict[int, str]] = {} # {time_step: {node_id: agent_id}}
    conflict_visuals: Dict[int, List[Tuple[int, int]]] = {} # {time: [(row, col), ...]}
    logs: List[str] = []
    results: List[AgentResult] = []

    initial_max_time_estimate = config.rows * config.cols # Generous upper bound for path length

    for i, agent_config in enumerate(config.agents):
        sid = node_id(*agent_config.start)
        gid = node_id(*agent_config.goal)

        initial_path_ids = reconstruct_path(sid, gid, path_matrix)
        
        conflict_detected_for_agent = False
        reroute_time = 0

        # Simulate agent's initial path to check for conflicts
        for t, node_id_on_path in enumerate(initial_path_ids):
            if t >= initial_max_time_estimate: # Prevent planning beyond MAX_TIME
                break 
            if node_id_on_path in time_window.get(t, {}):
                conflict_detected_for_agent = True
                conflict_agent_id = time_window[t][node_id_on_path]
                conflict_visuals.setdefault(t, []).append(id_to_coord(node_id_on_path))
                logs.append(f"⚠ Conflict at {id_to_coord(node_id_on_path)} on t={t} between {agent_config.id} and {conflict_agent_id}")
                reroute_time = t # Mark the time of conflict
                break # Conflict found, need to reroute

        if not conflict_detected_for_agent:
            # If no conflict, commit this agent's path to the time window
            for t, node_id_on_path in enumerate(initial_path_ids):
                if t >= initial_max_time_estimate: break
                time_window.setdefault(t, {})[node_id_on_path] = agent_config.id
            results.append(AgentResult(
                id=agent_config.id,
                path=path_to_coords(initial_path_ids),
                rerouted=False
            ))
        else:
            logs.append(f"🔄 Rerouting Agent {agent_config.id} from {id_to_coord(sid)} to {id_to_coord(gid)} due to conflict.")
            
            occupied_for_dijkstra = {
                t_step: set(nodes_at_t.keys()) for t_step, nodes_at_t in time_window.items()
            }
            
            reroute_start_node_id = initial_path_ids[reroute_time] 
            
            new_path_segment_ids = dijkstra_with_avoidance(reroute_start_node_id, gid, occupied_for_dijkstra, initial_max_time_estimate)
            
            if new_path_segment_ids and len(new_path_segment_ids) > 1:
                # The agent's *final* path is the original path up to reroute_time,
                # followed by the new Dijkstra path segment (which starts with reroute_start_node_id).
                # So, we take original path *before* reroute_time, and then append the new segment.
                final_path_ids = initial_path_ids[:reroute_time] + new_path_segment_ids

                # Update time_window with the *final_path_ids*
                for t, node_id_on_path in enumerate(final_path_ids):
                    # Use a generous limit for time window population, as rerouted paths can be long
                    if t >= initial_max_time_estimate * 3: 
                        break
                    time_window.setdefault(t, {})[node_id_on_path] = agent_config.id
                
                results.append(AgentResult(
                    id=agent_config.id,
                    path=path_to_coords(final_path_ids), # This is the full path the agent will take
                    rerouted=True,
                    reroute_time=reroute_time, # Time when rerouting was triggered
                    original_segment=path_to_coords(initial_path_ids[:reroute_time]), # Original path *before* reroute point
                    dijkstra_segment=path_to_coords(new_path_segment_ids) # The new segment from Dijkstra (starts at reroute node)
                ))
                logs.append(f"✅ Agent {agent_config.id} rerouted successfully. New path length: {len(final_path_ids)}")
            else:
                logs.append(f"❌ Agent {agent_config.id}: Could not find an alternative path. Agent might be stuck or wait.")
                results.append(AgentResult(
                    id=agent_config.id,
                    path=path_to_coords(initial_path_ids), # Keep original path if no reroute found
                    rerouted=False # Mark as not rerouted if no valid new path
                ))

    max_path_length = 0
    for agent_result in results:
        max_path_length = max(max_path_length, len(agent_result.path))
    
    return results, conflict_visuals, logs, max_path_length

# --- FastAPI App Setup ---

app = FastAPI(
    title="Warehouse Simulation API",
    description="API for simulating multi-agent path planning in a warehouse."
)

# Enable CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], # Allows all origins for development. Restrict in production.
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/simulate", response_model=SimulationResponse)
async def simulate_warehouse(config: SimulationConfig):
    """
    Runs the warehouse simulation based on the provided configuration.
    """
    try:
        agents_data, conflict_map, conflict_logs, max_time_for_animation = run_simulation(config)
        return SimulationResponse(
            agents_data=agents_data,
            conflict_map=conflict_map,
            conflict_logs=conflict_logs,
            max_time_for_animation=max_time_for_animation,
            grid_size=(config.rows, config.cols),
            shelves=config.shelves
        )
    except Exception as e:
        # Log the full traceback for debugging
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Simulation error: {str(e)}")
