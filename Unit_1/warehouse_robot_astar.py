import heapq


def heuristic(node, goal):
  """Calculates the Manhattan distance between two grid coordinates (x, y)."""
  return abs(node[0] - goal[0]) + abs(node[1] - goal[1])


def a_star_warehouse(grid, start, goal):
  """Finds the optimal path for a warehouse robot using A* Search.

  :param grid: 2D list where 0 represents open aisles and 1 represents storage
  racks/obstacles
  :param start: Tuple (x, y) representing the robot's starting position
  :param goal: Tuple (x, y) representing the target item/rack location
  :return: List of coordinates representing the optimal path, or None if no path
  exists
  """
  rows, cols = len(grid), len(grid[0])

  # Priority queue stores tuples of (f_cost, g_cost, current_position, path)
  # f_cost = g_cost + heuristic(current, goal)
  start_h = heuristic(start, goal)
  pq = [(start_h, 0, start, [start])]

  # Dictionary to store the minimum g_cost found for each node
  g_costs = {start: 0}

  # Possible movements: Up, Down, Left, Right
  directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

  while pq:
    f_cost, g_cost, current, path = heapq.heappop(pq)

    # Check if the robot has reached the goal
    if current == goal:
      return path

    # Explore neighboring cells
    for dx, dy in directions:
      neighbor = (current[0] + dx, current[1] + dy)
      nx, ny = neighbor

      # Check boundaries and ensure the cell is not an obstacle (1)
      if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
        new_g_cost = g_cost + 1  # Each step has a cost of 1

        # If this path to the neighbor is better than any previously recorded one
        if neighbor not in g_costs or new_g_cost < g_costs[neighbor]:
          g_costs[neighbor] = new_g_cost
          new_f_cost = new_g_cost + heuristic(neighbor, goal)
          new_path = path + [neighbor]
          heapq.heappush(pq, (new_f_cost, new_g_cost, neighbor, new_path))

  return None  # Return None if the goal is blocked or unreachable


# --- Example Simulation ---

# Warehouse Grid Layout:
# 0 = Open Aisle (Passable)
# 1 = Storage Rack / Obstacle (Impassable)
warehouse_grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0],
]

robot_start_pos = (0, 0)  # Top-left charging station
item_goal_pos = (4, 4)  # Bottom-right storage rack target

# Execute A* Search
optimal_path = a_star_warehouse(warehouse_grid, robot_start_pos, item_goal_pos)

# Output Results
if optimal_path:
  print('📦 Optimal Warehouse Robot Path Found (A* Search):')
  print(optimal_path)
  print(f'Total steps/distance: {len(optimal_path) - 1}')
else:
  print('⚠️ Alert: Target rack is blocked or unreachable!')