import networkx as nx
import numpy as np
import random
import memory_profiler as mem_profile


def generateRandomAdjAdjMatrixGraph(nodes, edges, weightRange):
    G = nx.Graph()

    num_nodes = nodes
    num_edges = edges

    G.add_nodes_from(range(num_nodes))
    nodes_added = 0

    while nodes_added < num_edges:
        node1 = random.choice(list(G.nodes))
        node2 = random.choice(list(G.nodes))
        if not G.has_edge(node1, node2):
            weight = random.randint(0, 10)
            G.add_edge(node1, node2, weight=weight)
            nodes_added += 1

    # Initialize an empty adjacency matrix
    adjacency_matrix = np.zeros((num_nodes, num_nodes))

    # Iterate over edges and fill in the adjacency matrix
    for node1, node2, weight in G.edges.data('weight'):
        adjacency_matrix[node1][node2] = weight
        adjacency_matrix[node2][node1] = weight

    return adjacency_matrix


def generateRandomAdjAdjList(nodes, edges, weightRange):
    # Create a directed random graph
    G = nx.Graph()

    num_nodes = nodes
    num_edges = edges

    G.add_nodes_from(range(num_nodes))
    nodes_added = 0

    while nodes_added < num_edges:
        node1 = random.choice(list(G.nodes))
        node2 = random.choice(list(G.nodes))
        if not G.has_edge(node1, node2):
            weight = random.randint(0, 10)
            G.add_edge(node1, node2, weight=weight)
            nodes_added += 1

    # Initialize an empty adjacency matrix
    adjacency_list = {}

    # Iterate over edges and fill in the adjacency matrix
    for node1, node2, weight in G.edges.data('weight'):
        if node1 not in adjacency_list:
            adjacency_list[node1] = []
        adjacency_list[node1].append((node2, weight))

        if node2 not in adjacency_list:
            adjacency_list[node2] = []
        adjacency_list[node2].append((node1, weight))

    return adjacency_list


# evaluate the performance of a function
# evaluation criteria:
#   - time
#   - memory
#   - storage
def evaluate(func, *args):
    import time

    # time
    start_time = time.time()
    func(*args)
    end_time = time.time()
    time_elapsed = end_time - start_time

    # convert time to human readable format
    # just miliseconds limit 2 decimal places
    time_elapsed = str(round(time_elapsed, 5)*1000) + " miliseconds"

    # memory
    # get memory usage before running the function
    mem_usage_beofore = mem_profile.memory_usage()[0]

    # get memory usage while running the function
    mem_usage = mem_profile.memory_usage(func)

    # take average of memory usage
    # TypeError: 'numpy.float64' object is not iterable
    mem_usage = sum(mem_usage) / len(mem_usage)

    # get the difference
    mem_usage = mem_usage - mem_usage_beofore

    # convert memory to human readable format (MB) limit 5 decimal places
    mem_usage = str(round(mem_usage, 5)) + " MB"

    print("Time elapsed: ", time_elapsed)
    print("Memory usage: ", mem_usage)
    return time_elapsed, mem_usage
