'''tkinter'''

import math
from ctypes import windll #not vital import, remove if too slow
from tkinter import *
from tkinter import messagebox
from tkinter import ttk
import time
import heapq
import networkx as nx
import osmnx as ox
from osmnx._errors import InsufficientResponseError
from requests.exceptions import ConnectionError
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)

LG = ('Verdana', 12, "bold")
SM = ('Verdana', 10)

class PriorityQueue:
    '''
    Priority Queue doctest
    '''
    def __init__(self):
        self._container = []

    def empty(self):
        '''insert doctest'''
        return len(self._container) == 0

    def put(self, item, priority):
        '''insert doctest'''
        heapq.heappush(self._container, (priority, item))

    def get(self):
        '''insert doctest'''
        return heapq.heappop(self._container)[1]

def error_handler(func: callable):
    '''Connection/Input error handler'''
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except ConnectionError:
            messagebox.showerror(
                'Connection Error', 'Please check your internet connection and try again'
                )
        except InsufficientResponseError:
            messagebox.showwarning('Querry Error', 'Unable to find the start or destination point. Please try again')
    return wrapper


def restoration(graph, end):
    '''insert doctest'''
    path = []
    this_node = end
    while this_node is not None:
        path.append(this_node)
        this_node = graph[this_node]
    path.reverse()
    return path

def dijkstra(loc_graph, start, end):
    '''
    dijkstra algorithm
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


    shortest_path = restoration(graph_for_path_restoration, end)
    return shortest_path, node_disctances[end]

def astar(graph, start, end):
    """
    Implements the A* algorithm for finding the shortest path in a graph.
    """
    node_distances = {node: math.inf for node in graph.nodes}
    heuristic_distances = {node: 0 for node in graph.nodes}
    path_restore = {node: None for node in graph.nodes}

    end_lat, end_lon = graph.nodes[end]['y'], graph.nodes[end]['x']

    for node in graph.nodes:
        lat, lon = graph.nodes[node]['y'], graph.nodes[node]['x']
        heuristic_distances[node] = ox.distance.great_circle(lat, lon, end_lat, end_lon)

    node_distances[start] = 0
    visited_nodes = set()

    queue = PriorityQueue()
    queue.put(start, heuristic_distances[start])

    while not queue.empty():
        curr_node = queue.get()

        if curr_node in visited_nodes:
            continue

        if curr_node == end:
            path = restoration(path_restore, end)
            return path, node_distances[end]

        visited_nodes.add(curr_node)

        curr_distance = node_distances[curr_node]

        for adj_node, attributes in graph[curr_node].items():
            edge_distance = attributes[0].get('length', 1)
            new_distance = curr_distance + edge_distance

            if new_distance < node_distances[adj_node]:
                node_distances[adj_node] = new_distance
                path_restore[adj_node] = curr_node

                # f(n) = g(n) + h(n) for priority in queue
                total_cost = new_distance + heuristic_distances[adj_node]
                queue.put(adj_node, total_cost)

    return None, math.inf




@error_handler
def shortest_distance(origin_point: str, destination_point: str, algorithm_type: str):
    '''main'''
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
    
    if algorithm_type == 'Dijkstra':
        shortest_path, distance_total = dijkstra(working_graph_area, node1, node2)
    else:
        shortest_path, distance_total = astar(working_graph_area, node1, node2)


    time_end = time.time()
    exec_time = time_end - time_start

    print(shortest_path, distance_total)
    return exec_time, distance_total, working_graph_area, shortest_path


def main():
    '''temp'''
    def handle_click():
        exec_time = 0
        shor_distance = 0
        method = value_string.get()
        start = e1.get()
        end = e2.get()

        print(method)

        if not start or not end:
            messagebox.showwarning('Error', 'Please enter a start point and try again.')
            return

        if method == 'Select an algorithm':
            messagebox.showwarning('Error', 'Please select an algorithm and try again.')
            return
        exec_time, shor_distance, working_distance, route = shortest_distance(start, end, method)
        duration.config(text=f"Time: {exec_time:.2f}s")
        distance.config(text=f"Time: {(shor_distance / 1000):.2f}km")
        ax.clear()
        ox.plot_graph(
            working_distance,
            node_size = 0,
            figsize = (5, 5),
            dpi = 100,
            show = False,
            ax = ax
            )
        ox.plot_graph_route(
            working_distance, route,
            node_size = 0,
            figsize = (5, 5),
            route_color='#007fff',
            dpi = 100,
            show = False,
            ax = ax
            )
        # ox.plot_grap

    root = Tk()
    windll.shcore.SetProcessDpiAwareness(1)
    side_nav = Frame(root)
    vis_frame = Frame(root)


    root.call("source", "Azure/azure.tcl")
    root.call("set_theme", "dark")

    entry_frame = Frame(side_nav, padx=20, width=240)
    entry_frame.grid_propagate(False)
    stats_frame = Frame(side_nav, padx=20, pady=40)
    value_string = StringVar(root, 'Select an algorithm..')

    ###################################
    fi = Figure((6,6), dpi=100)
    ax = fi.add_subplot()
    ax.axis('off')
    canvas = FigureCanvasTkAgg(fi, vis_frame)
    NavigationToolbar2Tk(canvas)
    h1 = Label(root, text='Find Shortest Path.', font=('Courier', 13, 'bold'))

    op1 = ttk.OptionMenu(entry_frame, value_string, 'Select an algorithm', 'Dijkstra', 'A*')

    l1 = Label(entry_frame, text='From', font=LG)
    l2 = Label(entry_frame, text='To', font=LG)

    e1 = ttk.Entry(entry_frame, font=LG)
    e2 = ttk.Entry(entry_frame, font=LG)

    duration = Label(stats_frame, text="Time:", font=SM)
    distance = Label(stats_frame, text="Distance:", font=SM)

    action = ttk.Button(
        entry_frame,
        text="Find shortest distance:",
        command=handle_click,
        )

    l1.pack(anchor='w', pady=5)
    e1.pack(ipady=5)
    l2.pack(anchor='w',  pady=5)
    e2.pack(ipady=5)
    h1.place(x=10, y=10)

    op1.pack(anchor='w', pady=25)
    action.pack(pady=15)
    duration.pack(anchor='w')
    distance.pack(pady=20, anchor='w')
    canvas.get_tk_widget().pack()


    ###################################


    ###################################
    entry_frame.pack()
    stats_frame.pack(anchor='sw')
    ###################################
    side_nav.grid(row=0, column=0)
    vis_frame.grid(row=0, column=1)
    root.mainloop()

main()
