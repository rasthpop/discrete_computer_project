'''Module provides Dijkstra algorythm'''

import math
from queue import PriorityQueue
import networkx as nx
import osmnx as ox


def dijkstra(loc_graph, start, end):
    '''
    
    '''
    node_disctances = {node:math.inf for node in loc_graph.nodes}
    node_disctances[start] = 0
    graph_for_path_restoration = {node:None for node in loc_graph.nodes}
    visited_nodes = set()

    priority = PriorityQueue()
    priority.put((0, start))

    while not priority.empty():
        current_distance, current_node = priority.get()

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
                priority.put((new_disctance, adjacent_node))
    path = []
    this_node = end
    while this_node is not None:
        path.append(this_node)
        this_node = graph_for_path_restoration[this_node]
    path.reverse()
    return path, node_disctances[end]


city1_name = 'Kyiv, Ukraine'
city2_name = 'Odessa, Ukraine'

city1_coords = ox.geocode(city1_name)
city2_coords = ox.geocode(city2_name)
airdistance = ox.distance.great_circle(city1_coords[0], city1_coords[1], \
                                       city2_coords[0], city2_coords[1])

graph_city1 = ox.graph_from_point(city1_coords, dist=airdistance, network_type='drive')
graph_city2 = ox.graph_from_point(city2_coords, dist=airdistance, network_type='drive')


working_graph_area = nx.compose(graph_city1, graph_city2)
node1 = ox.distance.nearest_nodes(working_graph_area, city1_coords[1], city2_coords[0])
node2 = ox.distance.nearest_nodes(working_graph_area, city2_coords[1], city2_coords[0])
shortest_path, distance_total = dijkstra(working_graph_area, node1, node2)
print(shortest_path, distance_total)