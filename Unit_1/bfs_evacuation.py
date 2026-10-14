from collections import deque


def bfs_evacuation_route(city_map, start_zone, evacuation_centers):
  """Finds the shortest evacuation route from a disaster zone to the nearest

  evacuation center using Breadth-First Search (BFS).

  :param city_map: Dictionary representing the adjacency list of city roads
  :param start_zone: The starting disaster location (string)
  :param evacuation_centers: Set of safe shelter locations
  :return: A list representing the shortest path from start to the nearest shelter
  """
  # Queue stores paths, starting with just the disaster zone
  queue = deque([[start_zone]])

  # Set to keep track of visited intersections to avoid loops
  visited = set([start_zone])

  while queue:
    # Dequeue the current path
    current_path = queue.popleft()
    current_intersection = current_path[-1]

    # Check if we have reached an evacuation center
    if current_intersection in evacuation_centers:
      return current_path

    # Explore all connected neighboring roads level by level
    for neighbor in city_map.get(current_intersection, []):
      if neighbor not in visited:
        visited.add(neighbor)
        # Create a new extended path and add it to the queue
        new_path = list(current_path)
        new_path.append(neighbor)
        queue.append(new_path)

  return None  # Return None if no route to any shelter exists


# --- Example Simulation ---

# City road network represented as an adjacency list
# Keys are intersections/zones, values are connected adjacent roads
city_road_network = {
    'Zone_A (Disaster Site)': ['Intersection_1', 'Intersection_2'],
    'Intersection_1': ['Zone_A (Disaster Site)', 'Intersection_3', 'Shelter_A'],
    'Intersection_2': ['Zone_A (Disaster Site)', 'Intersection_4'],
    'Intersection_3': ['Intersection_1', 'Shelter_B'],
    'Intersection_4': ['Intersection_2', 'Intersection_5'],
    'Intersection_5': ['Intersection_4'],
    'Shelter_A': ['Intersection_1'],
    'Shelter_B': ['Intersection_3'],
}

# Designated safe evacuation centers in the city
safe_shelters = {'Shelter_A', 'Shelter_B'}
disaster_start = 'Zone_A (Disaster Site)'

# Execute the BFS algorithm
safest_route = bfs_evacuation_route(
    city_road_network, disaster_start, safe_shelters
)

# Output Results
if safest_route:
  print('🚨 Safest Evacuation Route Found (Shortest Path):')
  print(' ➔ '.join(safest_route))
  print(f'Total road segments to cross: {len(safest_route) - 1}')
else:
  print('⚠️ Alert: No accessible evacuation route found from this zone!')