from collections import deque
import heapq
import time


def bfs_search(grid, start, goal):
  start_time = time.perf_counter()
  rows, cols = len(grid), len(grid[0])
  queue = deque([[start]])
  visited = {start}
  nodes_visited = 0
  directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

  while queue:
    path = queue.popleft()
    current = path[-1]
    nodes_visited += 1

    if current == goal:
      duration = time.perf_counter() - start_time
      return path, duration, nodes_visited

    for dx, dy in directions:
      nx, ny = current[0] + dx, current[1] + dy
      if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
        neighbor = (nx, ny)
        if neighbor not in visited:
          visited.add(neighbor)
          queue.append(path + [neighbor])

  duration = time.perf_counter() - start_time
  return None, duration, nodes_visited


def dfs_search(grid, start, goal):
  start_time = time.perf_counter()
  rows, cols = len(grid), len(grid[0])
  stack = [[start]]
  visited = set()
  nodes_visited = 0
  directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

  while stack:
    path = stack.pop()
    current = path[-1]

    if current not in visited:
      visited.add(current)
      nodes_visited += 1

      if current == goal:
        duration = time.perf_counter() - start_time
        return path, duration, nodes_visited

      for dx, dy in reversed(directions):
        nx, ny = current[0] + dx, current[1] + dy
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
          neighbor = (nx, ny)
          if neighbor not in visited:
            stack.append(path + [neighbor])

  duration = time.perf_counter() - start_time
  return None, duration, nodes_visited


def astar_search(grid, start, goal):
  start_time = time.perf_counter()
  rows, cols = len(grid), len(grid[0])

  def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

  pq = [(heuristic(start, goal), 0, start, [start])]
  g_costs = {start: 0}
  nodes_visited = 0
  directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

  while pq:
    f, g, current, path = heapq.heappop(pq)
    nodes_visited += 1

    if current == goal:
      duration = time.perf_counter() - start_time
      return path, duration, nodes_visited

    for dx, dy in directions:
      nx, ny = current[0] + dx, current[1] + dy
      if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
        neighbor = (nx, ny)
        new_g = g + 1
        if neighbor not in g_costs or new_g < g_costs[neighbor]:
          g_costs[neighbor] = new_g
          new_f = new_g + heuristic(neighbor, goal)
          heapq.heappush(pq, (new_f, new_g, neighbor, path + [neighbor]))

  duration = time.perf_counter() - start_time
  return None, duration, nodes_visited


# --- Shared Road Network Simulation ---
# 0 = Open Road, 1 = Blocked/Congested Road (Obstacle)
road_grid = [
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
]

start_location = (0, 0)
destination = (4, 5)

print('=============================================')
print('  NAVIGATION ALGORITHM PERFORMANCE EVALUATION')
print('=============================================')

algorithms = [('Breadth-First Search (BFS)', bfs_search), ('Depth-First Search (DFS)', dfs_search), ('A* Search', astar_search)]

for name, algo_func in algorithms:
  path, duration, nodes_visited = algo_func(road_grid, start_location, destination)
  path_length = (len(path) - 1) if path else 'No Path Found'

  print(f'\n-> Algorithm: {name}')
  print(f'   Path Length (Steps): {path_length}')
  print(f'   Nodes Explored:      {nodes_visited}')
  print(f'   Execution Time:      {duration:.6f} seconds')
  if path and name == 'DFS':
    print('   Note: DFS typically finds a winding, suboptimal path.')
    