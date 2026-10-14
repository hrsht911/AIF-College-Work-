import heapq


class HospitalDeliveryAgent:

  def __init__(self, hospital_map):
    """Initializes the intelligent agent with the hospital floor map.

    :param hospital_map: 2D list where 0 = open corridor, 1 = blocked corridor
    """
    self.map = hospital_map
    self.rows = len(hospital_map)
    self.cols = len(hospital_map[0])

  def _heuristic(self, node, goal):
    """Manhattan distance heuristic for grid navigation."""
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

  def plan_delivery_route(self, start_pos, target_department):
    """Goal-based planning: Computes the optimal path to the target department.

    :param start_pos: Tuple (x, y) of the robot's current location
    :param target_department: Tuple (x, y) of the destination department
    :return: List of coordinates representing the shortest safe path, or None
    """
    # Check if start or goal is blocked
    if (
        self.map[start_pos[0]][start_pos[1]] == 1
        or self.map[target_department[0]][target_department[1]] == 1
    ):
      print("⚠️ Error: Start or target department is inside a blocked corridor!")
      return None

    # Priority queue stores: (f_cost, g_cost, current_position, path)
    start_h = self._heuristic(start_pos, target_department)
    pq = [(start_h, 0, start_pos, [start_pos])]
    g_costs = {start_pos: 0}

    # Allowed movements: Up, Down, Left, Right
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while pq:
      f_cost, g_cost, current, path = heapq.heappop(pq)

      # Goal check
      if current == target_department:
        return path

      for dx, dy in directions:
        neighbor = (current[0] + dx, current[1] + dy)
        nx, ny = neighbor

        # Validate grid boundaries and check for blocked corridors (1)
        if 0 <= nx < self.rows and 0 <= ny < self.cols and self.map[nx][ny] == 0:
          new_g_cost = g_cost + 1

          if neighbor not in g_costs or new_g_cost < g_costs[neighbor]:
            g_costs[neighbor] = new_g_cost
            new_f_cost = new_g_cost + self._heuristic(neighbor, target_department)
            heapq.heappush(pq, (new_f_cost, new_g_cost, neighbor, path + [neighbor]))

    return None  # No path available

  def execute_delivery(self, start_pos, target_department, department_name):
    """Perception-Action loop for the delivery agent."""
    print(f"🤖 Agent initialized at {start_pos}.")
    print(f"🎯 Goal: Deliver medicine to {department_name} at {target_department}.")

    path = self.plan_delivery_route(start_pos, target_department)

    if path:
      print("✅ Optimal Route Planned Successfully!")
      print(f"Path sequence: {path}")
      print(f"Total corridor steps: {len(path) - 1}")
      print(f"📦 Status: Medicine successfully delivered to {department_name}!\n")
    else:
      print(
          f"❌ Status: Failed to reach {department_name}. All paths are"
          " blocked.\n"
      )


# --- Simulation Setup ---

# Hospital Floor Grid Layout:
# 0 = Open Corridor (Passable)
# 1 = Blocked Corridor / Construction / Hazard (Impassable)
hospital_floor_grid = [
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
]

# Instantiate the intelligent agent
robot_agent = HospitalDeliveryAgent(hospital_floor_grid)

# Define locations
pharmacy_start = (0, 0)
icu_department = (4, 5)
emergency_department = (0, 5)

# Execute missions
robot_agent.execute_delivery(pharmacy_start, icu_department, "ICU")
robot_agent.execute_delivery(pharmacy_start, emergency_department, "Emergency Room")