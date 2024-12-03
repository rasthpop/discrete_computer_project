'''Module provides Dijkstra algorythm'''

import math
import heapq
import networkx as nx
import osmnx as ox

class PriorityQueue:
    def __init__(self):
        self._container = []

    def empty(self):
        return len(self._container) == 0

    def put(self, item, priority):
        heapq.heappush(self._container, (priority, item))

    def get(self):
        return heapq.heappop(self._container)[1]


def dijkstra(loc_graph, start, end):
    '''
    
    '''
    node_disctances = {node:float('inf') for node in loc_graph.nodes}
    # print(node_disctances[end])
    node_disctances[start] = 0
    graph_for_path_restoration = {node:None for node in loc_graph.nodes}
    visited_nodes = set()
    # print(end in loc_graph.nodes)
    priority = PriorityQueue()
    priority.put(start, 0)

    while not priority.empty():
        current_node = priority.get()
        current_distance = node_disctances[current_node]
        # print(current_node, current_distance)
        if current_node in visited_nodes:
            # print(visited_nodes)
            continue
        visited_nodes.add(current_node)
        # if current_node == end:
        #     break
        for adjacent_node, attributes in loc_graph[current_node].items():
            # print(attributes.get('length', 1))
            edge_distance = attributes.get('length', 1)
            new_disctance = current_distance + edge_distance
            if new_disctance < node_disctances[adjacent_node]:
                node_disctances[adjacent_node] = new_disctance
                graph_for_path_restoration[adjacent_node] = current_node
                priority.put(adjacent_node, new_disctance)
        # print(node_disctances[current_node])
    # print(graph_for_path_restoration)
    print(node_disctances)
    path = []
    this_node = end
    while this_node is not None:
        path.append(this_node)
        # print(path)
        this_node = graph_for_path_restoration[this_node]
    path.reverse()
    
    return path, node_disctances[end]


origin_point = (50.4501, 30.5234)  # Центр Києва
destination_point = (50.4017, 30.3928)
place_name = "Kyiv, Ukraine"

# Створення графа для Києва
graph = ox.graph_from_place(place_name, network_type='drive')
origin_node = ox.distance.nearest_nodes(graph, origin_point[1], origin_point[0])
destination_node = ox.distance.nearest_nodes(graph, destination_point[1], destination_point[0])
# print(origin_node)
# print(destination_node)
shortest_path, distance_total = dijkstra(graph, origin_node, origin_node)
print(shortest_path, distance_total)
print(ox.shortest_path(graph, origin_node, destination_node, weight='length'))