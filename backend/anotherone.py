import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
from matplotlib.widgets import Slider # Import Slider

# Config
ROWS, COLS = 10, 10
MAX_TIME = 30
SHELVES = {
    (4, 4), (4, 5), (5, 4), (5, 5),
    (2, 2), (2, 3), (2, 4),
    (7, 6), (7, 7), (7, 8)
}
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Directions: Up, Down, Left, Right
AGENTS = [
    {"id": "A", "start": (0, 0), "goal": (9, 9)},
    {"id": "B", "start": (0, 9), "goal": (9, 0)},
    {"id": "C", "start": (9, 0), "goal": (0, 9)},
    {"id": "D", "start": (9, 9), "goal": (0, 0)},
    {"id": "E", "start": (5, 0), "goal": (5, 9)},
]
COLORS = ["red", "blue", "green", "orange", "purple"]

def in_bounds(x, y):
    """Checks if a given coordinate is within the grid boundaries."""
    return 0 <= x < ROWS and 0 <= y < COLS

def node_id(x, y):
    """Converts (x, y) coordinates to a unique node ID."""
    return x * COLS + y

def id_to_coord(id_):
    """Converts a node ID back to (x, y) coordinates."""
    return divmod(id_, COLS)

def build_graph():
    """
    Builds the adjacency matrix for the warehouse graph.
    Shelves are considered obstacles (no edges through them).
    """
    N = ROWS * COLS
    adj = np.full((N, N), np.inf) # Initialize with infinity for no connection
    for x in range(ROWS):
        for y in range(COLS):
            if (x, y) in SHELVES:
                continue # Cannot move through shelves
            a = node_id(x, y)
            adj[a][a] = 0 # Distance to self is 0

            for dx, dy in DIRS:
                nx, ny = x + dx, y + dy
                if in_bounds(nx, ny) and (nx, ny) not in SHELVES:
                    b = node_id(nx, ny)
                    adj[a][b] = 1 # Unit cost for moving to adjacent node
    return adj

def floyd_with_path(adj):
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

def reconstruct_path(i, j, path_matrix):
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


def path_to_coords(path_ids):
    """Converts a list of node IDs to a list of (x, y) coordinates."""
    return [id_to_coord(n) for n in path_ids]

def dijkstra_with_avoidance(start_id, goal_id, occupied_at_time):
    """
    Implements Dijkstra's algorithm to find a path from start to goal,
    avoiding nodes occupied by other agents at specific time steps.
    `occupied_at_time`: A dictionary where keys are time steps and values are sets of occupied node IDs.
    """
    visited = set() # Stores (node_id, time_step) to avoid cycles and re-exploring
    pq = [(0, start_id, [start_id])] # (cost, current_node_id, path_list)

    while pq:
        cost, current_id, path = heapq.heappop(pq)

        if current_id == goal_id:
            return path # Found shortest path

        # Check if this (node, time) state has already been visited with a shorter or equal path
        if (current_id, cost) in visited:
            continue
        visited.add((current_id, cost))

        x, y = id_to_coord(current_id)

        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if in_bounds(nx, ny) and (nx, ny) not in SHELVES:
                neighbor_id = node_id(nx, ny)
                next_time = cost + 1 # Each step takes 1 unit of time

                # Check for time window conflict: is the neighbor occupied at the next time step?
                if neighbor_id in occupied_at_time.get(next_time, set()):
                    continue # Skip this path if the next node is occupied

                heapq.heappush(pq, (next_time, neighbor_id, path + [neighbor_id]))
    return [start_id] # Return start node if no path found

def simulate_paths():
    """
    Simulates agent movement, detects conflicts, and recalculates paths.
    """
    global adj # Use global adj matrix for path reconstruction in Floyd
    adj = build_graph()
    dist, path_matrix = floyd_with_path(adj)
    
    # time_window: {time_step: {node_id: agent_id}}
    # This records the final planned occupancy for each node at each time step.
    time_window = {} 
    
    # conflict_visuals: {time_step: [(x,y), (x,y), ...]}
    # Stores coordinates of nodes where conflicts were detected for visualization.
    conflict_visuals = {} 
    
    logs = [] # Log messages for conflicts and rerouting
    results = [] # Final planned paths for all agents

    for i, agent_config in enumerate(AGENTS):
        sid = node_id(*agent_config["start"])
        gid = node_id(*agent_config["goal"])

        # Initial path calculation using Floyd-Warshall
        initial_path_ids = reconstruct_path(sid, gid, path_matrix)
        
        # Check for conflicts with already planned agents (time window)
        conflict_detected_for_agent = False
        reroute_time = 0 # The time at which the agent needs to be rerouted

        # Simulate agent's initial path to check for conflicts
        # We only check up to MAX_TIME to prevent infinite loops for very long paths
        for t, node_id_on_path in enumerate(initial_path_ids):
            if t >= MAX_TIME: # Prevent planning beyond MAX_TIME
                break 
            if node_id_on_path in time_window.get(t, {}):
                conflict_detected_for_agent = True
                conflict_agent_id = time_window[t][node_id_on_path]
                conflict_visuals.setdefault(t, []).append(id_to_coord(node_id_on_path))
                logs.append(f"⚠ Conflict at {id_to_coord(node_id_on_path)} on t={t} between {agent_config['id']} and {conflict_agent_id}")
                reroute_time = t # Mark the time of conflict
                break # Conflict found, need to reroute

        if not conflict_detected_for_agent:
            # If no conflict, commit this agent's path to the time window
            for t, node_id_on_path in enumerate(initial_path_ids):
                if t >= MAX_TIME: break
                time_window.setdefault(t, {})[node_id_on_path] = agent_config["id"]
            results.append({
                "id": agent_config["id"],
                "path": path_to_coords(initial_path_ids),
                "rerouted": False
            })
        else:
            # Conflict detected, recalculate path using Dijkstra
            logs.append(f"🔄 Rerouting Agent {agent_config['id']} from {id_to_coord(sid)} to {id_to_coord(gid)} due to conflict.")
            
            # Create the 'occupied' map for Dijkstra, considering all previously planned agents
            # This is crucial: Dijkstra needs to know what nodes are occupied at what future times.
            occupied_for_dijkstra = {
                t_step: set(nodes_at_t.keys()) for t_step, nodes_at_t in time_window.items()
            }
            
            # The agent needs to replan from its current position at `reroute_time`
            # For simplicity, we assume the agent is still at its last non-conflicting node
            # or its start if conflict is at t=0.
            # A more sophisticated model would track actual agent positions.
            # Here, we assume it's replanning from its original start node, but avoiding future conflicts.
            
            # The paper says "local Dijkstra algorithm is used to adjust the path within a localized area."
            # Our Dijkstra avoids specific nodes at specific times.
            
            new_path_ids = dijkstra_with_avoidance(sid, gid, occupied_for_dijkstra)
            
            # If a new path is found, commit it to the time window
            if new_path_ids and len(new_path_ids) > 1:
                for t, node_id_on_path in enumerate(new_path_ids):
                    if t >= MAX_TIME: break
                    time_window.setdefault(t, {})[node_id_on_path] = agent_config["id"]
                
                results.append({
                    "id": agent_config["id"],
                    "path": path_to_coords(new_path_ids),
                    "rerouted": True,
                    "reroute_time": reroute_time, # Time when rerouting was triggered
                    "original": path_to_coords(initial_path_ids[:reroute_time]) # Part of original path before reroute
                })
                logs.append(f"✅ Agent {agent_config['id']} rerouted successfully. New path length: {len(new_path_ids)}")
            else:
                logs.append(f"❌ Agent {agent_config['id']}: Could not find an alternative path. Agent might be stuck or wait.")
                # If no new path, agent effectively stays at its last valid position
                results.append({
                    "id": agent_config["id"],
                    "path": path_to_coords(initial_path_ids), # Keep original path if no reroute found
                    "rerouted": False # Mark as not rerouted if no valid new path
                })

    return results, conflict_visuals, logs

def animate_sim(agents_data, conflicts, logs):
    """
    Animates the simulation results using Matplotlib.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    plt.subplots_adjust(bottom=0.15) # Make space for the slider
    ax.set_xlim(-0.5, COLS - 0.5)
    ax.set_ylim(-0.5, ROWS - 0.5)
    ax.set_aspect("equal")
    plt.gca().invert_yaxis() # Invert y-axis to match typical grid (0,0) at top-left
    ax.set_title("Hybrid Floyd-Dijkstra Agent Routing")

    # Draw grid lines
    for x in range(COLS + 1):
        ax.axvline(x - 0.5, color='lightgray', lw=0.5)
    for y in range(ROWS + 1):
        ax.axhline(y - 0.5, color='lightgray', lw=0.5)

    # Draw shelves
    for y, x in SHELVES:
        ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="black", alpha=0.7))

    # Agent dots and labels
    dots = []
    labels = []
    path_lines = [] # One line object per agent, to be updated dynamically
    
    for i, agent in enumerate(agents_data):
        color = COLORS[i % len(COLORS)] # Cycle through colors
        
        # Initial position for the dot
        start_y, start_x = agent["path"][0] if agent["path"] else (0,0)
        dot, = ax.plot([start_x], [start_y], "o", color=color, markersize=12, zorder=5)
        
        # Initial position for the label
        label = ax.text(start_x, start_y, agent["id"], fontsize=9, ha='center', va='center', color='white', zorder=6)
        
        dots.append(dot)
        labels.append(label)
        
        # Initialize an empty line for each agent's path, will be updated in `update`
        line, = ax.plot([], [], color=color, linewidth=2, alpha=0.5) # Default style
        path_lines.append(line)

    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=12, verticalalignment='top')
    conflict_rects = [] # To store rectangles for conflicts

    # Timeline slider setup
    ax_slider = plt.axes([0.1, 0.05, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    time_slider = Slider(ax_slider, 'Time', 0, MAX_TIME - 1, valinit=0, valstep=1)

    def update(frame):
        """Update function for the animation."""
        # Clear previous conflict rectangles
        while conflict_rects:
            conflict_rects.pop().remove()

        # Add new conflict rectangles for the current frame
        if frame in conflicts:
            for y, x in conflicts[frame]:
                rect = plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color='red', alpha=0.5, zorder=4)
                ax.add_patch(rect)
                conflict_rects.append(rect)

        # Update agent positions, labels, and paths
        for i, agent in enumerate(agents_data):
            current_path_segment_coords = []
            current_line_style = '-'
            current_line_width = 2 # Default thickness for non-rerouted path
            current_line_alpha = 0.5

            if agent.get("rerouted"):
                reroute_time = agent["reroute_time"]
                original_segment_coords = agent["original"] # (y,x) coords of original path up to reroute_time-1
                new_full_path_coords = agent["path"] # (y,x) coords of the entire new path

                if frame < reroute_time:
                    # Show original path segment up to current frame
                    current_path_segment_coords = original_segment_coords[:frame + 1]
                    current_line_style = '-'
                    current_line_width = 2 # Original path thickness
                    current_line_alpha = 0.5
                else:
                    # Show the original path segment up to reroute_time, then the new path segment
                    path_before_reroute = original_segment_coords
                    
                    # Path segment after reroute (from the start of the new path)
                    idx_in_new_path = frame - reroute_time
                    if idx_in_new_path >= 0 and idx_in_new_path < len(new_full_path_coords):
                        path_after_reroute = new_full_path_coords[:idx_in_new_path + 1]
                    else:
                        path_after_reroute = new_full_path_coords # Show full new path if frame exceeds it

                    # Concatenate paths. `original_segment_coords` ends at the node *before* reroute_time.
                    # `new_full_path_coords` starts at the node *at* reroute_time.
                    # So, direct concatenation is correct.
                    current_path_segment_coords = path_before_reroute + path_after_reroute

                    current_line_style = '--' # Rerouted path is dashed
                    current_line_width = 3 # Rerouted path thickness (thicker)
                    current_line_alpha = 0.8
            else:
                # Not rerouted, show full precalculated path up to current frame
                full_path_coords = agent["path"]
                current_path_segment_coords = full_path_coords[:frame + 1]
                current_line_style = '-'
                current_line_width = 2 # Original path thickness
                current_line_alpha = 0.5

            # Update path line data
            if current_path_segment_coords:
                path_lines[i].set_data([x for _, x in current_path_segment_coords], 
                                       [y for y, _ in current_path_segment_coords])
                path_lines[i].set_linestyle(current_line_style)
                path_lines[i].set_linewidth(current_line_width)
                path_lines[i].set_alpha(current_line_alpha)
            else:
                path_lines[i].set_data([], []) # Hide line if no path segment

            # Update agent dot and label positions
            path = agent["path"]
            if frame < len(path):
                y, x = path[frame]
                dots[i].set_data([x], [y])
                labels[i].set_position((x, y))
            else:
                # Agent has reached its destination or simulation time exceeded path length
                # Keep agent at its last position
                if path:
                    y, x = path[-1]
                    dots[i].set_data([x], [y])
                    labels[i].set_position((x, y))
                else: # If path is empty (e.g., no path found)
                    dots[i].set_data([], []) # Hide dot
                    labels[i].set_text("") # Hide label

        time_text.set_text(f"Time: {frame}")
        
        # Return all artists that were modified
        return dots + labels + path_lines + [time_text] + conflict_rects

    # Connect slider to update function
    time_slider.on_changed(update)

    # Initial call to update to draw the first frame
    update(0)

    # FuncAnimation is still needed for blitting, but the slider drives the frames.
    ani = animation.FuncAnimation(fig, update, frames=MAX_TIME, interval=600, blit=False, repeat=False) # blit=False for dynamic artist management

    plt.legend()
    plt.grid(True)
    plt.show()

    print("\n--- Simulation Logs ---")
    print("\n".join(logs))

if __name__ == "__main__":
    # Ensure the global `adj` is available for `reconstruct_path`
    adj = build_graph() 
    
    agents_data, conflict_map, conflict_logs = simulate_paths()
    animate_sim(agents_data, conflict_map, conflict_logs)
