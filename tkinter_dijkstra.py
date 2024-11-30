'''tkinter'''

import math
from tkinter import ttk
from tkinter import *
from tkinter import messagebox
import time
import heapq
import networkx as nx
import osmnx as ox
from osmnx._errors import InsufficientResponseError
from requests.exceptions import ConnectionError
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)


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
            messagebox.showwarning(
                'Connection Error', 'Please check your internet connection and try again'
                )
        except InsufficientResponseError:
            messagebox.showwarning('meow', 'meow')
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


@error_handler
def shortest_distance(origin_point: str, destination_point: str):
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
    shortest_path, distance_total = dijkstra(working_graph_area, node1, node2)

    time_end = time.time()
    exec_time = time_end - time_start

    print(shortest_path, distance_total)
    return exec_time, distance_total, working_graph_area, shortest_path


def main():
    '''temp'''
    def handle_click():
        exec_time = 0
        shor_distance = 0
        start = e1.get()
        end = e2.get()

        if not start or not end:
            messagebox.showwarning('Error', 'Please enter a start point and try again.')
            return

        exec_time, shor_distance, working_distance, route = shortest_distance(start, end)
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
            dpi = 100,
            show = False,
            ax = ax
            )
        # ox.plot_grap

    root = Tk()
    side_nav = Frame(root)
    vis_frame = Frame(root)

    entry_frame = Frame(side_nav, padx=20, pady=20)
    stats_frame = Frame(side_nav, padx=20, pady=40)

    ################################### 
    fi = Figure((5,5), dpi=100)
    ax = fi.add_subplot()
    ax.axis('off')
    canvas = FigureCanvasTkAgg(fi, vis_frame)
    NavigationToolbar2Tk(canvas)

    h1 = Label(entry_frame, text='Find Shortest Path:')
    l1 = Label(entry_frame, text='From')
    l2 = Label(entry_frame, text='to')

    e1 = ttk.Entry(entry_frame)
    e2 = ttk.Entry(entry_frame)


    duration = Label(stats_frame, text="Time:")
    distance = Label(stats_frame, text="Distance:")

    action = Button(
        entry_frame, 
        pady=5, 
        padx=5, 
        text="Find shortest distance:", 
        command=handle_click
        )

    h1.pack(pady=10)
    l1.pack()
    e1.pack()
    l2.pack()
    e2.pack()
    action.pack(pady=20)
    duration.pack()
    distance.pack(pady=20)
    canvas.get_tk_widget().pack()


    ###################################


    ###################################
    entry_frame.grid(row=0,column=0)
    stats_frame.grid(row=1,column=0)
    ###################################
    side_nav.grid(row=0, column=0)
    vis_frame.grid(row=0, column=1)
    root.mainloop()

main()
