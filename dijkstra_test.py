'''Module provides Dijkstra algorythm'''

import math
import heapq
import networkx as nx
import time
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

def restoration(graph, end):
    path = []
    this_node = end
    while this_node is not None:
        path.append(this_node)
        this_node = graph[this_node]
    path.reverse()
    return path

def dijkstra(loc_graph, start, end):
    '''
    dijkstra
    '''

    node_disctances = {node: math.inf for node in loc_graph.nodes}
    node_disctances[start] = 0
    graph_for_path_restoration = {node: None for node in loc_graph.nodes}
    visited_nodes = set()

    priority = PriorityQueue()
    priority.put(start, 0)

    while not priority.empty():
        current_node = priority.get()
        current_distance = node_disctances[current_node]

        if current_node in visited_nodes:
            continue
        visited_nodes.add(current_node)
        if current_node == end:
            break

        for adjacent_node, attributes in loc_graph[current_node].items():
            edge_distance = attributes[0].get('length', 1)
            new_disctance = current_distance + edge_distance
            if new_disctance < node_disctances[adjacent_node]:
                node_disctances[adjacent_node] = new_disctance
                graph_for_path_restoration[adjacent_node] = current_node
                priority.put(adjacent_node, new_disctance)
    # path = []
    # this_node = end
    # while this_node is not None:
    #     path.append(this_node)
    #     this_node = graph_for_path_restoration[this_node]
    # path.reverse()
    shortest_path = restoration(graph_for_path_restoration, end)
    return shortest_path, node_disctances[end]



def main():
    '''main'''
    start = time.time()
    origin_point = (50.4501, 30.5234)
    destination_point = (50.4017, 30.3928)
    place_name = "Kyiv, Ukraine"

    graph = ox.graph_from_place(place_name, network_type='drive')
    origin_node = ox.distance.nearest_nodes(graph, origin_point[1], origin_point[0])
    destination_node = ox.distance.nearest_nodes(graph, destination_point[1], destination_point[0])

    shortest1_path, distance_total = dijkstra(graph, origin_node, destination_node)
    end = time.time()

    exec_time = end - start
    print(len(shortest1_path), distance_total)
    print(len(ox.shortest_path(graph, origin_node, destination_node, weight='length')))
    print(round(exec_time ,2))
    


main()

# city1_name = 'Kyiv, Ukraine'
# city2_name = 'Chernihiv, Ukraine'

# city1_coords = ox.geocode(city1_name)
# city2_coords = ox.geocode(city2_name)
# airdistance = ox.distance.great_circle(city1_coords[0], city1_coords[1], \
#                                        city2_coords[0], city2_coords[1])

# graph_city1 = ox.graph_from_point(city1_coords, dist=airdistance/2, network_type='drive')
# graph_city2 = ox.graph_from_point(city2_coords, dist=airdistance/2, network_type='drive')


# working_graph_area = nx.compose(graph_city1, graph_city2)
# node1 = ox.distance.nearest_nodes(working_graph_area, city1_coords[1], city1_coords[0])
# node2 = ox.distance.nearest_nodes(working_graph_area, city2_coords[1], city2_coords[0])
# shortest_path, distance_total = dijkstra(working_graph_area, node1, node2)
# print(shortest_path, distance_total)
