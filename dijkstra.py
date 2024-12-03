'''
Module implements Dijkstra's algorithm
'''

import math
import heapq
import time
import networkx as nx
import osmnx as ox


def track_time(func):
    """
    Decorator to track and print the execution time of a function.

    Args:
        func: The function to decorate.

    Returns:
        wrapper function that tracks execution time.
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"Execution time of {func.__name__}: {
              execution_time:.6f} seconds")
        return result

    return wrapper


class PriorityQueue:
    def __init__(self):
        self._container = []

    def is_empty(self):
        return len(self._container) == 0

    def put(self, item, priority):
        heapq.heappush(self._container, (priority, item))

    def get(self):
        return heapq.heappop(self._container)[1]


def path_restoration(path_restore, curr_node):
    '''
    Reconstruct the path by following the nodes from goal to start.
    '''
    path = []
    while curr_node is not None:
        path.append(curr_node)
        curr_node = path_restore[curr_node]
    path.reverse()
    return path

@track_time
def dijkstra(graph, start, end):
    """
    Implements Dijkstra's algorithm for finding the shortest path.
    """
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
            edge_distance = attributes[0].get('length', 1)
            new_distance = curr_distance + edge_distance

            if new_distance < node_distances[adj_node]:
                node_distances[adj_node] = new_distance
                path_restore[adj_node] = curr_node
                queue.put(adj_node, new_distance)

    path = path_restoration(path_restore, end)
    return path, node_distances[end]



def main():
    '''
    Main function
    '''
    origin_point = (50.4501, 30.5234)
    destination_point = (50.4017, 30.3928)
    place_name = "Kyiv, Ukraine"

    curr_graph = ox.graph_from_place(place_name, network_type='drive')

    origin_node = ox.distance.nearest_nodes(
        curr_graph, origin_point[1], origin_point[0])
    destination_node = ox.distance.nearest_nodes(
        curr_graph, destination_point[1], destination_point[0])

    shortest_path, distance_total = dijkstra(
        curr_graph, origin_node, destination_node)

    print(f"Path length: {len(shortest_path)} nodes")
    print(f"Total distance: {distance_total} meters")

    print(f"OSMnx shortest path length: {len(ox.shortest_path(
        curr_graph, origin_node, destination_node, weight='length'))}")


if __name__ == "__main__":
    main()
