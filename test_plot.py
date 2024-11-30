# import matplotlib.pyplot as plt
# import networkx as nx
# import osmnx as ox

# city = input("enter city name >>> ")

# city1 = ox.geocode(city)
# res_graph = ox.graph_from_point(city1, 6000)

# ox.plot_graph(res_graph, node_size=0)
# plt.show()

def error_handler(func: callable):
    '''Connection/Input error handler'''
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            print('This wrapper (probably) works fine')
            
    return wrapper


@error_handler
def some_function(number1, number2):
    return number1 + number2

some_function([1], 2)