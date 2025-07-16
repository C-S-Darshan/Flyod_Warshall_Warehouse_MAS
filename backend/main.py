from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Tuple, Dict, Any
import numpy as np
import heapq

# --- Pydantic Models for API Request/Response ---

class Agent(BaseModel):
    id: str
    start: Tuple[int, int]
    goal: Tuple[int, int]

class SimulationRequest(BaseModel):
    rows: int
    cols: int
    shelves: List[Tuple[int, int]]
    agents: List[Agent]

# --- FastAPI App Initialization ---

app = FastAPI()

# Configure CORS to allow requests from the React frontend
origins = [
    "http://localhost:3000",
    "http://localhost:3001",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Core Simulation Logic (Adapted from the script) ---

# Helper functions
def in_bounds(r, c, rows, cols):
    return 0 <= r < rows and 0 <= c < cols

def node_id(r, c, cols):
    return r * cols + c

def id_to_coord(id_, cols):
    """
    Converts a node ID back to (row, col) coordinates.
    Ensures the output is a standard Python int tuple.
    """
    r, c = divmod(id_, cols)
    return (int(r), int(c))

# Graph and Pathfinding
def build_graph(rows, cols, shelves_set):
    N = rows * cols
    adj = np.full((N, N), np.inf)
    COST_WITH_FLOW = 1.0
    COST_AGAINST_FLOW = 1.5

    for r in range(rows):
        for c in range(cols):
            if (r, c) in shelves_set:
                continue
            
            current_node_id = node_id(r, c, cols)
            adj[current_node_id, current_node_id] = 0

            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nr, nc = r + dr, c + dc
                if in_bounds(nr, nc, rows, cols) and (nr, nc) not in shelves_set:
                    neighbor_id = node_id(nr, nc, cols)
                    cost = COST_WITH_FLOW
                    if dr == 1 and c % 2 != 0: cost = COST_AGAINST_FLOW
                    elif dr == -1 and c % 2 == 0: cost = COST_AGAINST_FLOW
                    elif dc == 1 and r % 2 != 0: cost = COST_AGAINST_FLOW
                    elif dc == -1 and r % 2 == 0: cost = COST_AGAINST_FLOW
                    adj[current_node_id, neighbor_id] = cost
    return adj

def floyd_with_path(adj):
    N = adj.shape[0]
    dist = adj.copy()
    next_node_matrix = np.full((N, N), -1, dtype=int)
    for i in range(N):
        for j in range(N):
            if adj[i, j] != np.inf and i != j:
                next_node_matrix[i, j] = j
    for k in range(N):
        for i in range(N):
            for j in range(N):
                if dist[i, k] != np.inf and dist[k, j] != np.inf and dist[i, k] + dist[k, j] < dist[i, j]:
                    dist[i, j] = dist[i, k] + dist[k, j]
                    next_node_matrix[i, j] = next_node_matrix[i, k]
    return dist, next_node_matrix

def reconstruct_path(start_id, goal_id, next_node_matrix):
    if next_node_matrix[start_id, goal_id] == -1: return []
    path = [start_id]
    current_id = start_id
    while current_id != goal_id:
        current_id = next_node_matrix[current_id, goal_id]
        if current_id == -1: return []
        path.append(current_id)
    return path

def dijkstra_with_avoidance(start_id, goal_id, start_time, occupied_at_time, max_time, rows, cols, shelves_set):
    pq = [(start_time, start_id, [start_id])]
    visited = set()
    while pq:
        cost, current_id, path = heapq.heappop(pq)
        if current_id == goal_id: return path
        if (current_id, cost) in visited: continue
        visited.add((current_id, cost))
        r, c = id_to_coord(current_id, cols)
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if in_bounds(nr, nc, rows, cols) and (nr, nc) not in shelves_set:
                neighbor_id = node_id(nr, nc, cols)
                next_time = cost + 1
                if next_time >= max_time * 2: continue
                if neighbor_id in occupied_at_time.get(next_time, set()): continue
                heapq.heappush(pq, (next_time, neighbor_id, path + [neighbor_id]))
    return None

# Main Simulation Orchestrator
def run_simulation_logic(rows: int, cols: int, shelves: List[Tuple[int, int]], agents: List[Agent]):
    shelves_set = set(map(tuple, shelves))
    adj = build_graph(rows, cols, shelves_set)
    _, next_node_matrix = floyd_with_path(adj)
    
    time_window = {}
    conflict_markers = {}
    logs = []
    final_agent_paths = []
    max_path_len_estimate = rows * cols

    for agent_config in agents:
        agent_id = agent_config.id
        start_id = node_id(*agent_config.start, cols)
        goal_id = node_id(*agent_config.goal, cols)
        initial_path_ids = reconstruct_path(start_id, goal_id, next_node_matrix)
        
        if not initial_path_ids:
            logs.append(f"❌ Agent {agent_id}: No initial path found.")
            final_agent_paths.append({"id": agent_id, "path": [], "rerouted": False})
            continue

        conflict_time = -1
        for t, node in enumerate(initial_path_ids):
            if node in time_window.get(t, {}):
                conflict_time = t
                conflict_coord = id_to_coord(node, cols)
                conflicting_agent = time_window[t][node]
                logs.append(f"⚠️ Conflict for Agent {agent_id} at {conflict_coord} (t={t}) with {conflicting_agent}.")
                conflict_markers.setdefault(t, []).append(conflict_coord)
                break
        
        final_path_ids = []
        is_rerouted = False
        reroute_data = {}

        if conflict_time == -1:
            logs.append(f"✅ Agent {agent_id}: Initial path is conflict-free.")
            final_path_ids = initial_path_ids
        else:
            reroute_start_time = max(0, conflict_time - 1)
            reroute_start_node = initial_path_ids[reroute_start_time]
            logs.append(f"🔄 Rerouting Agent {agent_id} from t={reroute_start_time}.")
            occupied_for_dijkstra = {t: set(nodes.keys()) for t, nodes in time_window.items()}
            
            new_path_segment = dijkstra_with_avoidance(
                reroute_start_node, goal_id, reroute_start_time, occupied_for_dijkstra, max_path_len_estimate, rows, cols, shelves_set
            )
            
            if new_path_segment:
                original_segment = initial_path_ids[:reroute_start_time]
                final_path_ids = original_segment + new_path_segment
                is_rerouted = True
                reroute_data = {
                    "reroute_time": reroute_start_time,
                    "original_segment": [id_to_coord(n, cols) for n in original_segment],
                    "dijkstra_segment": [id_to_coord(n, cols) for n in new_path_segment]
                }
                logs.append(f"✅ Agent {agent_id} rerouted successfully.")
            else:
                logs.append(f"❌ Rerouting failed for Agent {agent_id}. Agent will wait.")
                wait_time = 1
                path_before = initial_path_ids[:conflict_time]
                final_path_ids = path_before + [path_before[-1]] * wait_time + initial_path_ids[conflict_time:]
        
        for t, node in enumerate(final_path_ids):
            time_window.setdefault(t, {})[node] = agent_id
        
        agent_result = {
            "id": agent_id,
            "path": [id_to_coord(n, cols) for n in final_path_ids],
            "rerouted": is_rerouted,
            "initial_path": [id_to_coord(n, cols) for n in initial_path_ids]
        }
        if is_rerouted:
            agent_result.update(reroute_data)
        final_agent_paths.append(agent_result)

    max_time = max((len(p["path"]) for p in final_agent_paths if p["path"]), default=0)
    
    # Ensure all keys in conflicts are standard Python ints
    final_conflicts = {int(k): v for k, v in conflict_markers.items()}

    return {
        "agents_data": final_agent_paths,
        "conflicts": final_conflicts,
        "logs": logs,
        "max_time": int(max_time) # Explicitly cast to standard int
    }

# --- API Endpoint ---

@app.post("/run_simulation")
async def create_simulation(request: SimulationRequest) -> Dict[str, Any]:
    """
    Receives grid and agent configuration, runs the simulation,
    and returns the results.
    """
    return run_simulation_logic(request.rows, request.cols, request.shelves, request.agents)
