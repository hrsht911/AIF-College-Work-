def dfs_mine_inspection(mine_map, start_tunnel, goal_tunnel):
  """Finds a path from the start tunnel to the goal tunnel using Depth First Search (DFS).

  Simulates an autonomous robot exploring a mine tunnel completely.

  :param mine_map: Dictionary representing the mine tunnel network (adjacency
  list)
  :param start_tunnel: Starting location of the robot (string)
  :param goal_tunnel: Target inspection destination (string)
  :return: List representing the path taken by the robot
  """
  # Stack stores paths for DFS (Last-In, First-Out)
  stack = [[start_tunnel]]

  # Set to keep track of inspected junctions to prevent infinite loops
  visited = set()

  while stack:
    # Pop the most recent path (dives deep into the latest branch)
    current_path = stack.pop()
    current_tunnel = current_path[-1]

    # Check if the robot reached the target goal
    if current_tunnel == goal_tunnel:
      return current_path

    # If the tunnel hasn't been fully explored yet
    if current_tunnel not in visited:
      visited.add(current_tunnel)

      # Explore connected tunnels.
      # Reversing the list ensures we process them in a natural left-to-right order.
      for neighbor in reversed(mine_map.get(current_tunnel, [])):
        if neighbor not in visited:
          new_path = list(current_path)
          new_path.append(neighbor)
          stack.append(new_path)

  return None  # Return None if no path exists to the goal


# --- Example Simulation ---

# Mine tunnel network represented as an adjacency list
# Each junction connects to various sub-tunnels or inspection points
mine_tunnel_network = {
    'Surface_Entrance': ['Tunnel_Alpha', 'Tunnel_Beta'],
    'Tunnel_Alpha': ['Surface_Entrance', 'Junction_1', 'Junction_2'],
    'Junction_1': ['Tunnel_Alpha', 'Deep_Shaft_A'],
    'Deep_Shaft_A': ['Junction_1'],
    'Junction_2': ['Tunnel_Alpha', 'Target_Chamber'],
    'Target_Chamber': ['Junction_2'],
    'Tunnel_Beta': ['Surface_Entrance', 'Junction_3'],
    'Junction_3': ['Tunnel_Beta'],
}

robot_start = 'Surface_Entrance'
inspection_target = 'Target_Chamber'

# Execute the DFS algorithm
robot_path = dfs_mine_inspection(
    mine_tunnel_network, robot_start, inspection_target
)

# Output Results
if robot_path:
  print('🤖 Autonomous Inspection Route Found (DFS - Deep Dive):')
  print(' ➔ '.join(robot_path))
  print(f'Total junctions traversed: {len(robot_path) - 1}')
else:
  print('⚠️ Alert: Target inspection chamber is unreachable!')