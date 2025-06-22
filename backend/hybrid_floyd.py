import heapq
import numpy as np

GRID_ROWS = 10
GRID_COLS = 10
MAX_TIME = 100

DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right


def in_bounds(x, y):
    return 0 <= x < GRID_ROWS and 0 <= y < GRID_COLS


def node_id(x, y):
    return x * GRID_COLS + y


def id_to_coord(id_):
    return divmod(id_, GRID_COLS)


def build_graph():
    N = GRID_ROWS * GRID_COLS
    adj = np.full((N, N), np.inf)

    # Add shelves as obstacles
    shelves = {(4, 4), (4, 5), (5, 4), (5, 5)}  # block these 4 cells

    for x in range(GRID_ROWS):
        for y in range(GRID_COLS):
            if (x, y) in shelves:
                continue
            a = node_id(x, y)
            for dx, dy in DIRS:
                nx, ny = x + dx, y + dy
                if in_bounds(nx, ny) and (nx, ny) not in shelves:
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
        left = reconstruct_path(i, k, path)[:-1]
        right = reconstruct_path(k, j, path)
        return left + right


def path_to_coords(path_ids):
    return [list(id_to_coord(n)) for n in path_ids]


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
            if in_bounds(nx, ny):
                nid = node_id(nx, ny)
                t = len(path)
                if t >= MAX_TIME:
                    continue
                if nid in occupied.get(t, set()):
                    continue
                heapq.heappush(pq, (cost + 1, nid, path + [nid]))
    return [start]  # fallback: stay in place


def compute_paths(data):
    agents = data["agents"]
    adj = build_graph()
    dist, path = floyd_with_path(adj)
    time_window = {}  # {time: {node: agent_id}}
    conflict_visuals = {}  # {time: set of conflicting node_ids}
    results = []

    for agent in agents:
        agent_id = agent["id"]
        sx, sy = agent["start"]
        gx, gy = agent["goal"]

        start = node_id(sx, sy)
        goal = node_id(gx, gy)

        path_ids = reconstruct_path(start, goal, path)
        conflict = False

        for t, node in enumerate(path_ids):
            if t not in time_window:
                time_window[t] = {}
            if node in time_window[t]:
                conflict = True
                if t not in conflict_visuals:
                    conflict_visuals[t] = set()
                conflict_visuals[t].add(node)
                break
            time_window[t][node] = agent_id

        if not conflict:
            final_path = path_to_coords(path_ids)
        else:
            # reroute with local Dijkstra avoiding occupied nodes
            occupied = {t: set(time_window.get(t, {}).keys()) for t in range(MAX_TIME)}
            new_path_ids = dijkstra_with_avoidance(start, goal, occupied)
            for t, node in enumerate(new_path_ids):
                if t not in time_window:
                    time_window[t] = {}
                time_window[t][node] = agent_id
            final_path = path_to_coords(new_path_ids)

        results.append({
            "id": agent_id,
            "path": final_path
        })

    visual_conflicts = {
        int(t): [list(id_to_coord(int(n))) for n in nodes]
        for t, nodes in conflict_visuals.items()
    }

    return {
        "agents": json_safe(results),
        "conflicts": json_safe(visual_conflicts)
    }

# -----------------------------
# JSON-safe conversion helpers
# -----------------------------

def json_safe(obj):
    if isinstance(obj, dict):
        return {json_safe(k): json_safe(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [json_safe(i) for i in obj]
    elif isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    else:
        return obj
