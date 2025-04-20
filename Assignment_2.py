import time
import heapq

# Representasi grid kota
city_map = [
    ['S', '.', '.', '.', 'T', '.', '.', '.', '.', '.'],
    ['.', 'T', 'T', '.', 'T', '.', 'T', 'T', '.', '.'],
    ['.', '.', '.', '.', '.', '.', '.', '.', 'T', '.'],
    ['T', 'T', 'T', 'T', '.', 'T', '.', '.', 'T', '.'],
    ['.', '.', '.', '.', '.', '.', '.', '.', '.', 'H']
]

ROWS = len(city_map)
COLS = len(city_map[0])
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # atas, bawah, kiri, kanan

def find_position(symbol):
    for r in range(ROWS):
        for c in range(COLS):
            if city_map[r][c] == symbol:
                return (r, c)
    return None

def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def search(grid, start, goal, method='gbfs'):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored_nodes = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        explored_nodes += 1

        if current == goal:
            break

        for dr, dc in DIRECTIONS:
            nr, nc = current[0] + dr, current[1] + dc
            next_cell = (nr, nc)

            if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] != 'T':
                new_cost = cost_so_far[current] + 1

                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    heuristic = manhattan(next_cell, goal)

                    if method == 'gbfs':
                        priority = heuristic
                    elif method == 'astar':
                        priority = new_cost + heuristic
                    else:
                        raise ValueError("Gunakan metode 'gbfs' atau 'astar'.")

                    heapq.heappush(frontier, (priority, next_cell))
                    came_from[next_cell] = current

    path = reconstruct_path(came_from, start, goal)
    return path, explored_nodes

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        if current is None:
            return []
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# ==========================
# Eksekusi dan Perbandingan
# ==========================
if __name__ == "__main__":
    start = find_position('S')
    goal = find_position('H')

    results = {}

    for algo in ['gbfs', 'astar']:
        start_time = time.time()
        path, explored = search(city_map, start, goal, method=algo)
        end_time = time.time()
        elapsed = (end_time - start_time) * 1000

        results[algo] = {
            "path": path,
            "time_ms": elapsed,
            "nodes": explored
        }

        print(f"\nMetode: {algo.upper()}")
        if path:
            print("Path:", path)
            print(f"Time : {elapsed:.4f} ms")
        else:
            print("Path tidak ditemukan.")

    # 📊 Ringkasan Perbandingan
    print("\n📊 Perbandingan Waktu & Jumlah Node:")
    print(f"A*   : {results['astar']['time_ms']:.4f} ms, {results['astar']['nodes']} node")
    print(f"GBFS : {results['gbfs']['time_ms']:.4f} ms, {results['gbfs']['nodes']} node")
