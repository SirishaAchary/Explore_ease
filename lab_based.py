import heapq
def read_graph_from_file(filename):
    graph = {}
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split()
            node1, node2, weight = parts[0], parts[1], int(parts[2])
            if node1 not in graph:
                graph[node1] = {}
            if node2 not in graph:
                graph[node2] = {}
            graph[node1][node2] = weight
            graph[node2][node1] = weight  # Assuming undirected graph
    return graph

def floyd_warshall(graph):
    # Initialize the distance matrix with maximum possible value
    distance = {v: {u: float('inf') for u in graph} for v in graph}
    for v in graph:
        distance[v][v] = 0  # Distance from a vertex to itself is 0

    # Update distance matrix with direct edges
    for u in graph:
        for v in graph[u]:
            distance[u][v] = graph[u][v]

    # Apply Floyd-Warshall algorithm
    for k in graph:
        for i in graph:
            for j in graph:
                distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j])

    return distance

def find_closest_destination(source, destinations, distance_matrix):
    closest_destination = None
    min_distance = float('inf')
    for destination in destinations:
        if distance_matrix[source][destination] < min_distance:
            min_distance = distance_matrix[source][destination]
            closest_destination = destination
    return closest_destination, min_distance


def dijkstra(graph, start):
    # Initialize distances to all vertices as infinity
    distances = {vertex: float('infinity') for vertex in graph}
    distances[start] = 0

    # Keep track of predecessors to reconstruct the path
    predecessors = {vertex: None for vertex in graph}

    # Priority queue to keep track of vertices to visit
    pq = [(0, start)]

    while pq:
        current_distance, current_vertex = heapq.heappop(pq)

        # If we have already found a shorter path to current_vertex, ignore it
        if current_distance > distances[current_vertex]:
            continue

        # Explore neighbors of current_vertex
        for neighbor, weight in graph[current_vertex].items():
            distance = current_distance + weight
            # If new distance is shorter than the known distance, update it
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_vertex
                heapq.heappush(pq, (distance, neighbor))

    return distances, predecessors


def shortest_path(graph, start, end):
    distances, predecessors = dijkstra(graph, start)

    # Reconstruct the shortest path
    path = []
    current_vertex = end
    while current_vertex is not None:
        path.insert(0, current_vertex)
        current_vertex = predecessors[current_vertex]

    return path


def main():
    filename = 'graph.txt'
    filename.strip()
    graph = read_graph_from_file(filename)
    distance_matrix = floyd_warshall(graph)
    print("Distance Matrix:")
    for row in distance_matrix:
        print(row, distance_matrix[row])
    print()

    print("Enter from this location set")
    print("Gopalpur, Tampara, Taratarini, Balakumari, Sonepur, Ramlingeswar, Gandhinagar, Gokuldham, Bhairabi, NewBusStand")
    source = input("Enter the source node: ").strip()
    destinations = input("Enter the destination nodes (separated by space): ").strip().split()
    source1 = source

    #shortest = []
    # for destination in destinations:
    #     shortest = shortest_path(graph, source, destination)
    #     print("Shortest path from", source, "to", destination, ":", shortest)

    time_arr = []
    time_available = int(input("enter the total time you have(in minutes):"))
    for i in destinations:
        time_arr.append(int(input(f'enter the time you want to spend in destination {i} : ')))

    total_destination_time = sum(time_arr)
    #print(total_destination_time)

    path = []
    total_distance = 0
    while destinations:
        closest_destination, min_distance = find_closest_destination(source1, destinations, distance_matrix)
        path.append((closest_destination, min_distance))
        total_distance += min_distance
        destinations.remove(closest_destination)
        source1 = closest_destination

    path_time = total_distance * 2
    total_time = path_time + sum(time_arr)

    new_path = []
    if total_time <= time_available:
        print("\nOptimal Path:")
        for node, distance in path:
            print(f"{node} ({distance} units)", end=" -> ")
            new_path.append(node)
        print(f"\nTotal Distance: {total_distance} units")
        print(total_time)
        #print(new_path)
    else:
        print("no you cannot cover all destination.")

    shortest = []
    for destination in new_path:
        shortest = shortest_path(graph, source, destination)
        print("Shortest path from", source, "to", destination, ":", shortest)
        source = destination

if __name__== "__main__":
    main()