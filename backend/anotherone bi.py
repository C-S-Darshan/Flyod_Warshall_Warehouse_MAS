import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
from matplotlib.widgets import Slider

# --- CONFIGURATION ---
ROWS, COLS = 10, 10
SHELVES = {
    (2, 2), (2, 3), 
    (4, 1), (4, 2),
    (4, 6), (4, 7), (4, 8),
    (7, 2), (7, 3),
    (7, 6), (7, 7)
}
# Using the original agent list that was previously impossible
AGENTS = [
    {"id": "A", "start": (0, 0), "goal": (9, 9)},
    {"id": "B", "start": (0, 9), "goal": (9, 0)},
    {"id": "C", "start": (5, 0), "goal": (5, 9)},
    {"id": "D", "start": (9, 4), "goal": (0, 4)},
    {"id": "E", "start": (8, 0), "goal": (1, 8)},
]
COLORS = ["red", "blue", "green", "orange", "purple", "brown", "pink"]

# --- HELPER FUNCTIONS ---

def in_bounds(r, c):
    """Checks if a given coordinate is within the grid boundaries."""
    return 0 <= r < ROWS and 0 <= c < COLS

def node_id(r, c):
    """Converts (row, col) coordinates to a unique node ID."""
    return r * COLS + c

def id_to_coord(id_):
    """Converts a node ID back to (row, col) coordinates."""
    return divmod(id_, COLS)

# --- GRAPH AND PATHFINDING LOGIC ---

def build_graph():
    """
    Builds an adjacency matrix where all 4 moves are possible, but moving
    against the designated "flow" of a lane has a higher cost. This ensures
    all nodes are reachable while still encouraging efficient routing.
    """
    N = ROWS * COLS
    adj = np.full((N, N), np.inf)
    
    COST_WITH_FLOW = 1.0  # Lower cost for following the lane rule
    COST_AGAINST_FLOW = 1.5 # Higher cost for going against the lane rule

    for r in range(ROWS):
        for c in range(COLS):
            if (r, c) in SHELVES:
                continue
            
            current_node_id = node_id(r, c)
            adj[current_node_id, current_node_id] = 0

            # Consider all four possible directions
            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]: # Right, Left, Down, Up
                nr, nc = r + dr, c + dc

                if in_bounds(nr, nc) and (nr, nc) not in SHELVES:
                    neighbor_id = node_id(nr, nc)
                    cost = COST_WITH_FLOW
                    
                    # Check if the move is against the flow and assign higher cost
                    if dr == 1: # Moving Down...
                        if c % 2 != 0: cost = COST_AGAINST_FLOW # ...in an 'up' column
                    elif dr == -1: # Moving Up...
                        if c % 2 == 0: cost = COST_AGAINST_FLOW # ...in a 'down' column
                    elif dc == 1: # Moving Right...
                        if r % 2 != 0: cost = COST_AGAINST_FLOW # ...in a 'left' row
                    elif dc == -1: # Moving Left...
                        if r % 2 == 0: cost = COST_AGAINST_FLOW # ...in a 'right' row
                    
                    adj[current_node_id, neighbor_id] = cost
    return adj

def floyd_with_path(adj):
    """
    Implements the Floyd-Warshall algorithm.
    Returns the distance matrix and the 'next_node' matrix for path reconstruction.
    """
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
                if dist[i, k] != np.inf and dist[k, j] != np.inf:
                    if dist[i, k] + dist[k, j] < dist[i, j]:
                        dist[i, j] = dist[i, k] + dist[k, j]
                        next_node_matrix[i, j] = next_node_matrix[i, k]
    return dist, next_node_matrix

def reconstruct_path(start_id, goal_id, next_node_matrix):
    """
    Reconstructs the shortest path from a start to a goal node
    using the 'next_node' matrix from Floyd-Warshall.
    """
    if next_node_matrix[start_id, goal_id] == -1:
        return [] # No path exists

    path = [start_id]
    current_id = start_id
    while current_id != goal_id:
        current_id = next_node_matrix[current_id, goal_id]
        if current_id == -1: return [] # Should not happen if path exists
        path.append(current_id)
    return path

def path_to_coords(path_ids):
    """Converts a list of node IDs to a list of (row, col) coordinates."""
    return [id_to_coord(n) for n in path_ids]

def dijkstra_with_avoidance(start_id, goal_id, start_time, occupied_at_time, max_time):
    """
    Finds a path from start to goal, avoiding nodes occupied by other agents.
    This is a time-aware Dijkstra's algorithm (or A* with h=0).
    - occupied_at_time: {time_step: {occupied_node_id, ...}}
    """
    pq = [(start_time, start_id, [start_id])] # (cost, current_node_id, path_list)
    visited = set() # Stores (node_id, time_step)

    while pq:
        cost, current_id, path = heapq.heappop(pq)

        if current_id == goal_id:
            return path

        if (current_id, cost) in visited:
            continue
        visited.add((current_id, cost))

        r, c = id_to_coord(current_id)
        
        # Consider all physically possible moves from the current node
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            
            if in_bounds(nr, nc) and (nr, nc) not in SHELVES:
                neighbor_id = node_id(nr, nc)
                next_time = cost + 1

                if next_time >= max_time * 2: # Limit search to prevent long detours
                    continue
                
                # Check for time-window conflict
                if neighbor_id in occupied_at_time.get(next_time, set()):
                    continue

                heapq.heappush(pq, (next_time, neighbor_id, path + [neighbor_id]))
    
    return None # No path found

# --- SIMULATION ORCHESTRATOR ---

def simulate_paths():
    """
    Orchestrates the hybrid simulation:
    1. Pre-calculates all paths with Floyd-Warshall.
    2. Sequentially plans for each agent.
    3. Checks for conflicts in the time-window.
    4. Reroutes with Dijkstra if a conflict is found.
    """
    adj = build_graph()
    _, next_node_matrix = floyd_with_path(adj)
    
    time_window = {}
    conflict_markers = {}
    logs = []
    final_agent_paths = []
    
    max_path_len_estimate = ROWS * COLS 

    for agent_config in AGENTS:
        agent_id = agent_config["id"]
        start_id = node_id(*agent_config["start"])
        goal_id = node_id(*agent_config["goal"])

        initial_path_ids = reconstruct_path(start_id, goal_id, next_node_matrix)
        
        if not initial_path_ids:
            logs.append(f"❌ Agent {agent_id}: No initial path found from {agent_config['start']} to {agent_config['goal']}.")
            final_agent_paths.append({"id": agent_id, "path": [], "rerouted": False})
            continue

        conflict_time = -1
        for t, node in enumerate(initial_path_ids):
            if node in time_window.get(t, {}):
                conflict_time = t
                conflict_coord = id_to_coord(node)
                conflicting_agent = time_window[t][node]
                logs.append(f"⚠️ Conflict predicted for Agent {agent_id} at {conflict_coord} (t={t}) with Agent {conflicting_agent}.")
                conflict_markers.setdefault(t, []).append(conflict_coord)
                break
        
        final_path_ids = []
        is_rerouted = False
        reroute_data = {}

        if conflict_time == -1: # No conflict
            logs.append(f"✅ Agent {agent_id}: Initial path is conflict-free.")
            final_path_ids = initial_path_ids
            is_rerouted = False
        else: # Conflict found, must reroute
            reroute_start_time = conflict_time - 1
            if reroute_start_time < 0: reroute_start_time = 0 # handle conflict at t=0
            reroute_start_node = initial_path_ids[reroute_start_time]
            
            logs.append(f"🔄 Rerouting Agent {agent_id} from node {id_to_coord(reroute_start_node)} at t={reroute_start_time}.")

            occupied_for_dijkstra = {t: set(nodes.keys()) for t, nodes in time_window.items()}
            
            new_path_segment = dijkstra_with_avoidance(
                reroute_start_node, goal_id, reroute_start_time, occupied_for_dijkstra, max_path_len_estimate
            )
            
            if new_path_segment:
                original_segment = initial_path_ids[:reroute_start_time]
                final_path_ids = original_segment + new_path_segment
                is_rerouted = True
                reroute_data = {
                    "reroute_time": reroute_start_time,
                    "original_segment": path_to_coords(original_segment),
                    "dijkstra_segment": path_to_coords(new_path_segment)
                }
                logs.append(f"✅ Agent {agent_id} rerouted successfully. New path length: {len(final_path_ids)}.")
            else: 
                logs.append(f"❌ Rerouting failed for Agent {agent_id}. Agent will wait at conflict point.")
                wait_time = 1 
                path_before_conflict = initial_path_ids[:conflict_time]
                path_after_conflict = initial_path_ids[conflict_time:]
                final_path_ids = path_before_conflict + [path_before_conflict[-1]] * wait_time + path_after_conflict
                is_rerouted = False 
        
        for t, node in enumerate(final_path_ids):
            time_window.setdefault(t, {})[node] = agent_id
        
        agent_result = {
            "id": agent_id,
            "path": path_to_coords(final_path_ids),
            "rerouted": is_rerouted,
        }
        if is_rerouted:
            agent_result.update(reroute_data)
        
        final_agent_paths.append(agent_result)

    max_time = 0
    if final_agent_paths:
        max_time = max((len(p["path"]) for p in final_agent_paths if p["path"]), default=0)
        
    return final_agent_paths, conflict_markers, logs, max_time, next_node_matrix

# --- VISUALIZATION ---

def animate_sim(agents_data, conflicts, logs, max_time, next_node_matrix):
    """
    Animates the simulation results using Matplotlib.
    """
    fig, ax = plt.subplots(figsize=(12, 12))
    plt.subplots_adjust(bottom=0.15, left=0.05, right=0.95)
    ax.set_xlim(-0.5, COLS - 0.5)
    ax.set_ylim(-0.5, ROWS - 0.5)
    ax.set_aspect("equal")
    plt.gca().invert_yaxis()
    ax.set_title("Hybrid Floyd-Dijkstra Warehouse Simulation (Bidirectional)")
    ax.set_xticks(np.arange(-0.5, COLS, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, ROWS, 1), minor=True)
    ax.grid(which='minor', color='lightgray', linestyle='-', linewidth=0.5)

    for r, c in SHELVES:
        ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1, color="black", alpha=0.8, zorder=2))

    for i, agent_config in enumerate(AGENTS):
        color = COLORS[i % len(COLORS)]
        start_id = node_id(*agent_config["start"])
        goal_id = node_id(*agent_config["goal"])
        
        initial_floyd_ids = reconstruct_path(start_id, goal_id, next_node_matrix)
        if initial_floyd_ids:
            coords = path_to_coords(initial_floyd_ids)
            ax.plot([c for r, c in coords], [r for r, c in coords], 
                    color=color, alpha=0.3, linestyle=':', linewidth=2, zorder=1)

    agent_dots = []
    agent_labels = []
    solid_path_lines = []
    dashed_path_lines = []
    
    for i, agent in enumerate(agents_data):
        color = COLORS[i % len(COLORS)]
        start_r, start_c = agent["path"][0] if agent["path"] else (-1, -1)
        
        dot, = ax.plot([start_c], [start_r], "o", color=color, markersize=14, zorder=5)
        label = ax.text(start_c, start_r, agent["id"], fontsize=9, ha='center', va='center', color='white', zorder=6, weight='bold')
        solid_line, = ax.plot([], [], color=color, linewidth=2.5, alpha=0.7, zorder=3)
        dashed_line, = ax.plot([], [], color=color, linewidth=2.5, linestyle='--', alpha=0.9, zorder=4)
        
        agent_dots.append(dot)
        agent_labels.append(label)
        solid_path_lines.append(solid_line)
        dashed_path_lines.append(dashed_line)

    time_text = ax.text(0.01, 0.98, "", transform=ax.transAxes, fontsize=14, verticalalignment='top', bbox=dict(boxstyle='round,pad=0.3', fc='wheat', alpha=0.5))
    conflict_rects = []

    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.03], facecolor='lightgoldenrodyellow')
    time_slider = Slider(ax_slider, 'Time', 0, max_time -1, valinit=0, valstep=1)

    def update(frame):
        frame = int(frame)
        while conflict_rects:
            conflict_rects.pop().remove()

        if frame in conflicts:
            for r, c in conflicts[frame]:
                rect = plt.Rectangle((c - 0.5, r - 0.5), 1, 1, color='red', alpha=0.6, zorder=4, ec='black')
                ax.add_patch(rect)
                conflict_rects.append(rect)

        for i, agent in enumerate(agents_data):
            path = agent["path"]
            if not path: continue

            pos_idx = min(frame, len(path) - 1)
            r, c = path[pos_idx]
            agent_dots[i].set_data([c], [r])
            agent_labels[i].set_position((c, r))
            
            if agent["rerouted"]:
                reroute_time = agent["reroute_time"]
                solid_segment_end = min(frame + 1, reroute_time + 1)
                solid_coords = agent["original_segment"][:solid_segment_end]
                if solid_coords:
                    solid_path_lines[i].set_data([sc for sr, sc in solid_coords], [sr for sr, sc in solid_coords])
                
                if frame >= reroute_time:
                    dashed_segment_end = frame - reroute_time + 1
                    dashed_coords = agent["dijkstra_segment"][:dashed_segment_end]
                    if dashed_coords:
                         dashed_path_lines[i].set_data([dc for dr, dc in dashed_coords], [dr for dr, dc in dashed_coords])
                else: 
                    dashed_path_lines[i].set_data([], [])

            else: 
                path_to_draw = path[:frame + 1]
                solid_path_lines[i].set_data([sc for sr, sc in path_to_draw], [sr for sr, sc in path_to_draw])
        
        time_text.set_text(f"Time: {frame}")
        fig.canvas.draw_idle()

    time_slider.on_changed(update)
    update(0) 

    plt.show()

    print("\n--- Simulation Logs ---")
    print("\n".join(logs))


if __name__ == "__main__":
    agents_data, conflict_map, logs, max_time_val, next_node_m = simulate_paths()
    if max_time_val > 0:
        animate_sim(agents_data, conflict_map, logs, max_time_val, next_node_m)
    else:
        print("Simulation could not run as no paths were found for any agent.")