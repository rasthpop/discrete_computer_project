'''
CMD interface via argpargse
'''
from argparse import ArgumentParser, Namespace
import math
import heapq
import time
import networkx as nx
import osmnx as ox
from osmnx._errors import InsufficientResponseError
from requests.exceptions import ConnectionError



class PriorityQueue:
    def __init__(self):
        self._container = []

    def is_empty(self):
        return len(self._container) == 0

    def put(self, item, priority):
        heapq.heappush(self._container, (priority, item))

    def get(self):
        return heapq.heappop(self._container)[1]
# def track_time(func):
#     """
#     Decorator to track and print the execution time of a function.

#     Args:
#         func: The function to decorate.

#     Returns:
#         wrapper function that tracks execution time.
#     """
#     def wrapper(*args, **kwargs):
#         start_time = time.time()
#         result = func(*args, **kwargs)
#         end_time = time.time()
#         execution_time = end_time - start_time
#         print(f"Execution time of {func.__name__}: {
#               execution_time:.6f} seconds")
#         return result

#     return wrapper

def error_handler(func: callable):
    '''Connection/Input error handler'''
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except ConnectionError:
            print('connection error', 'please check your internet connection and try again.')

        except InsufficientResponseError:
            print('querry error', 'unable to find the start or destination point. Please try again.')
    return wrapper



def path_restoration(path_restore, curr_node):
    '''
    Reconstructs the path from the goal node to the start node.
    '''
    path = []
    while curr_node is not None:
        path.append(curr_node)
        curr_node = path_restore[curr_node]
    path.reverse()
    return path


def astar(graph, start, end):
    """
    Implements the A* algorithm for finding the shortest path in a graph.
    """
    node_distances = {node: math.inf for node in graph.nodes}
    heuristic_distances = {node: 0 for node in graph.nodes}
    path_restore = {node: None for node in graph.nodes}
    node_counter = 0

    end_lat, end_lon = graph.nodes[end]['y'], graph.nodes[end]['x']

    for node in graph.nodes:
        lat, lon = graph.nodes[node]['y'], graph.nodes[node]['x']
        heuristic_distances[node] = ox.distance.great_circle(lat, lon, end_lat, end_lon)

    node_distances[start] = 0
    visited_nodes = set()

    queue = PriorityQueue()
    queue.put(start, heuristic_distances[start])

    while not queue.is_empty():
        curr_node = queue.get()

        if curr_node in visited_nodes:
            continue

        if curr_node == end:
            return node_distances[end], node_counter

        visited_nodes.add(curr_node)

        curr_distance = node_distances[curr_node]

        for adj_node, attributes in graph[curr_node].items():
            node_counter += 1
            edge_distance = attributes[0].get('length', 1)
            new_distance = curr_distance + edge_distance

            if new_distance < node_distances[adj_node]:
                node_distances[adj_node] = new_distance
                path_restore[adj_node] = curr_node

                # f(n) = g(n) + h(n) for priority in queue
                total_cost = new_distance + heuristic_distances[adj_node]
                queue.put(adj_node, total_cost)

    return None, math.inf

def dijkstra(graph, start, end):
    """
    Implements Dijkstra's algorithm for finding the shortest path.
    """
    counter = 0
    node_distances, path_restore = {}, {}
    for node in graph.nodes:
        node_distances[node] = math.inf
        path_restore[node] = None

    node_distances[start] = 0
    visited_nodes = set()

    queue = PriorityQueue()
    queue.put(start, 0)

    while not queue.is_empty():
        curr_node = queue.get()

        if curr_node in visited_nodes:
            continue
        visited_nodes.add(curr_node)

        curr_distance = node_distances[curr_node]

        for adj_node, attributes in graph[curr_node].items():
            counter += 1
            edge_distance = attributes[0].get('length', 1)
            new_distance = curr_distance + edge_distance

            if new_distance < node_distances[adj_node]:
                node_distances[adj_node] = new_distance
                path_restore[adj_node] = curr_node
                queue.put(adj_node, new_distance)

    return node_distances[end], counter



@error_handler
def shortest_distance(origin_point: str, destination_point: str, algorithm_type: str):
    '''main'''
    if algorithm_type not in {'dijkstra', 'a*'}:
        return "incorrect algorithm choice. possible choices: dijkstra, a*"

    time_start = time.time()

    city1_coords = ox.geocode(origin_point)
    city2_coords = ox.geocode(destination_point)



    airdistance = ox.distance.great_circle(city1_coords[0], city1_coords[1], \
                                        city2_coords[0], city2_coords[1])

    graph_city1 = ox.graph_from_point(city1_coords, dist=airdistance/2, network_type='drive')
    graph_city2 = ox.graph_from_point(city2_coords, dist=airdistance/2, network_type='drive')

    working_graph_area = nx.compose(graph_city1, graph_city2)
    node1 = ox.distance.nearest_nodes(working_graph_area, city1_coords[1], city1_coords[0])
    node2 = ox.distance.nearest_nodes(working_graph_area, city2_coords[1], city2_coords[0])

    if algorithm_type == 'dijkstra':
        distance_total, nodes_count = dijkstra(working_graph_area, node1, node2)
    elif algorithm_type == 'a*':
        distance_total, nodes_count = astar(working_graph_area, node1, node2)
    else:
        return "incorrect algorithm choice. possible choices: dijkstra, a*"

    time_end = time.time()

    exec_time = time_end - time_start

    # print(shortest_path, distance_total)
    res_string = f"{"-"*9}\nNodes: {nodes_count}\nDistance: {(distance_total / 1000):.2f}km\ntime: {exec_time}\n{"-"*9}"
    return res_string

# # @track_time
# def main():
#     '''
#     Main function
#     '''
#     origin_point = (50.4501, 30.5234)
#     destination_point = (50.4017, 30.3928)
#     place_name = "Kyiv, Ukraine"

#     curr_graph = ox.graph_from_place(place_name, network_type='drive')

#     origin_node = ox.distance.nearest_nodes(
#         curr_graph, origin_point[1], origin_point[0])
#     destination_node = ox.distance.nearest_nodes(
#         curr_graph, destination_point[1], destination_point[0])

#     shortest_path, distance_total = astar(
#         curr_graph, origin_node, destination_node)

#     print(f"Path length: {len(shortest_path)} nodes")
#     print(f"Total distance: {distance_total} meters")

#     print(f"OSMnx shortest path length: {len(ox.shortest_path(
#         curr_graph, origin_node, destination_node, weight='length'))}")


parser = ArgumentParser()


parser.add_argument('starting_point', type=str, help='The starting point from where the algorithm begins')
parser.add_argument('destination', type=str, help='The destination point that will be connected with the starting point')
parser.add_argument('algorithm', type=str, help="Type of algorithm that will be used to find the shortest path (dijkstra | a*)")
parser.add_argument(
                    '-f', '--findspath', 
                    action='store_true',
                    help="Finds the shortest path between two cities",

                    )
args: Namespace = parser.parse_args()
result: str = shortest_distance(args.starting_point, args.destination, args.algorithm)

if args.findspath:
    print(result)
else:
    print(result)
