import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq

# Config
ROWS, COLS = 10, 10
MAX_TIME = 30
SHELVES = {
    (4, 4), (4, 5), (5, 4), (5, 5),
    (2, 2), (2, 3), (2, 4),
    (7, 6), (7, 7), (7, 8)
}
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
AGENTS = [
    {"id": "A", "start": (0, 0), "goal": (9, 9)},
    {"id": "B", "start": (0, 9), "goal": (9, 0)},
    {"id": "C", "start": (9, 0), "goal": (0, 9)},
    {"id": "D", "start": (9, 9), "goal": (0, 0)},
    {"id": "E", "start": (5, 0), "goal": (5, 9)},
]
COLORS = ["red", "blue", "green", "orange", "purple"]

def in_bounds(x, y):
    return 0 <= x < ROWS and 0 <= y < COLS

def node_id(x, y):
    return x * COLS + y

def id_to_coord(id_):
    return divmod(id_, COLS)

def build_graph():
    N = ROWS * COLS
    adj = np.full((N, N), np.inf)
    for x in range(ROWS):
        for y in range(COLS):
            if (x, y) in SHELVES:
                continue
            a = node_id(x, y)
            for dx, dy in DIRS:
                nx, ny = x + dx, y + dy
                if in_bounds(nx, ny) and (nx, ny) not in SHELVES:
                    b = node_id(nx, ny)
                    adj[a][b] = 1
    return adj

def floyd_with_path(adj):
    N = adj.shape[0]
    dist = adj.copy()
    path = np.full((N, N), -1)
    for k in range(N):
        for i in range(N):
            for j in range(N):
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    path[i][j] = k
    return dist, path

def reconstruct_path(i, j, path):
    k = path[i][j]
    if k == -1:
        return [i, j] if i != j else [i]
    else:
        return reconstruct_path(i, k, path)[:-1] + reconstruct_path(k, j, path)

def path_to_coords(path_ids):
    return [id_to_coord(n) for n in path_ids]

def dijkstra_with_avoidance(start, goal, occupied):
    visited = set()
    pq = [(0, start, [start])]
    while pq:
        cost, current, path = heapq.heappop(pq)
        if current == goal:
            return path
        if (current, len(path)) in visited:
            continue
        visited.add((current, len(path)))
        x, y = id_to_coord(current)
        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if in_bounds(nx, ny) and (nx, ny) not in SHELVES:
                nid = node_id(nx, ny)
                t = len(path)
                if nid in occupied.get(t, set()):
                    continue
                heapq.heappush(pq, (cost + 1, nid, path + [nid]))
    return [start]

def simulate_paths():
    adj = build_graph()
    dist, path = floyd_with_path(adj)
    time_window = {}
    conflict_visuals = {}
    logs = []
    results = []

    for i, agent in enumerate(AGENTS):
        sid = node_id(*agent["start"])
        gid = node_id(*agent["goal"])
        path_ids = reconstruct_path(sid, gid, path)
        conflict = False

        for t, node in enumerate(path_ids):
            if t not in time_window:
                time_window[t] = {}
            if node in time_window[t]:
                conflict = True
                conflict_visuals.setdefault(t, []).append(id_to_coord(node))
                logs.append(f"⚠ Conflict at {id_to_coord(node)} on t={t} between {agent['id']} and {time_window[t][node]}")
                break
            time_window[t][node] = agent["id"]

        if not conflict:
            results.append({
                "id": agent["id"],
                "path": path_to_coords(path_ids),
                "rerouted": False
            })
        else:
            occupied = {t: set(time_window.get(t, {}).keys()) for t in range(MAX_TIME)}
            new_path_ids = dijkstra_with_avoidance(sid, gid, occupied)
            for t, node in enumerate(new_path_ids):
                time_window.setdefault(t, {})[node] = agent["id"]
            reroute_time = len(path_ids[:t])
            results.append({
                "id": agent["id"],
                "path": path_to_coords(new_path_ids),
                "rerouted": True,
                "reroute_time": reroute_time,
                "original": path_to_coords(path_ids[:reroute_time])
            })
    return results, conflict_visuals, logs

def animate_sim(agents, conflicts, logs):
    fig, ax = plt.subplots()
    ax.set_xlim(-0.5, COLS - 0.5)
    ax.set_ylim(-0.5, ROWS - 0.5)
    ax.set_aspect("equal")
    plt.gca().invert_yaxis()
    ax.set_title("Hybrid Floyd-Dijkstra Agent Routing")

    for x in range(COLS + 1):
        ax.axvline(x - 0.5, color='lightgray', lw=0.5)
    for y in range(ROWS + 1):
        ax.axhline(y - 0.5, color='lightgray', lw=0.5)

    for y, x in SHELVES:
        ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="black"))

    dots = []
    labels = []
    for i, agent in enumerate(agents):
        color = COLORS[i]
        dot, = ax.plot([], [], "o", color=color, markersize=12)
        label = ax.text(0, 0, agent["id"], fontsize=9, ha='center', va='center')
        dots.append(dot)
        labels.append(label)

        if agent.get("rerouted"):
            pre = agent["original"]
            post = agent["path"][agent["reroute_time"]:]
            ax.plot([x for _, x in pre], [y for y, _ in pre], color=color, alpha=0.3)
            ax.plot([x for _, x in post], [y for y, _ in post], color=color, linestyle="dashed", alpha=0.8)
        else:
            ax.plot([x for _, x in agent["path"]], [y for y, _ in agent["path"]], color=color, alpha=0.3)

    time_text = ax.text(0, -1, "", fontsize=12)
    conflict_rects = []

    def update(frame):
        while conflict_rects:
            conflict_rects.pop().remove()

        if frame in conflicts:
            for y, x in conflicts[frame]:
                rect = plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color='red', alpha=0.5)
                ax.add_patch(rect)
                conflict_rects.append(rect)

        for i, agent in enumerate(agents):
            path = agent["path"]
            if frame < len(path):
                y, x = path[frame]
                dots[i].set_data([x], [y])
                labels[i].set_position((x, y))

        time_text.set_text(f"Time: {frame}")
        return dots + labels + [time_text] + conflict_rects

    ani = animation.FuncAnimation(fig, update, frames=MAX_TIME, interval=600, blit=True, repeat=False)
    plt.show()

    print("\n".join(logs))

if __name__ == "__main__":
    agents_data, conflict_map, conflict_logs = simulate_paths()
    animate_sim(agents_data, conflict_map, conflict_logs)
