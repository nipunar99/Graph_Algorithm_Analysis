from utils import generateRandomAdjAdjMatrixGraph, generateRandomAdjAdjList
import numpy as np


class GraphAdjMatrix:
    def __init__(self, vertices, edges, random=False):
        self.V = vertices

        if random:
            self.graph = generateRandomAdjAdjMatrixGraph(vertices, edges, 10)
        else:
            self.graph = np.zeros((vertices, vertices))

    def addEdge(self, u, v, w):
        self.graph[u][v] = w
        self.graph[v][u] = w

    def getEdges(self):
        edges = []
        for i in range(self.V):
            for j in range(self.V):
                if self.graph[i][j] > 0:
                    edges.append((i, j, self.graph[i][j]))
        return edges

    # adj matric looks like [[],[],[]]
    def kruskelForAdjMatrix(self):
        result = []
        i, e = 0, 0

        # Create a list of edges as tuples (u, v, w)
        edges = self.getEdges()

        # Sort the list of edges by weight
        edges.sort(key=lambda item: item[2])

        parent = list(range(self.V))
        rank = [0] * self.V

        while e < self.V - 1:
            if i >= len(edges):
                break
            u, v, w = edges[i]
            i = i + 1
            x = self.find(parent, u)
            y = self.find(parent, v)

            if x != y:
                e = e + 1
                result.append((u, v, w))
                self.union(parent, rank, x, y)

        return result

    def find(self, parent, i):
        if parent[i] == i:
            return i
        return self.find(parent, parent[i])

    def union(self, parent, rank, x, y):
        xroot = self.find(parent, x)
        yroot = self.find(parent, y)

        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot

        else:
            parent[yroot] = xroot
            rank[xroot] += 1

    # adj matric looks like [[],[],[]]
    def primsLazy(self):
        # Initialize the minimum spanning tree and a set to keep track of visited vertices
        mst = []
        visited = set()

        # Choose a starting vertex (you can choose any vertex, I'll choose 0 here)
        start_vertex = 0

        # Create a priority queue to store edges with their weights
        edge_queue = []

        # Add all edges connected to the starting vertex to the priority queue
        for v in range(self.V):
            if self.graph[start_vertex][v] > 0:
                edge_queue.append(
                    (start_vertex, v, self.graph[start_vertex][v]))

        # Mark the starting vertex as visited
        visited.add(start_vertex)

        # Use a heap data structure to efficiently retrieve the minimum-weight edge
        import heapq
        heapq.heapify(edge_queue)

        while edge_queue:
            u, v, w = heapq.heappop(edge_queue)

            if v in visited:
                continue

            mst.append((u, v, w))
            visited.add(v)

            # Add all edges connected to vertex v to the priority queue
            for vertex in range(self.V):
                if self.graph[v][vertex] > 0 and vertex not in visited:
                    heapq.heappush(
                        edge_queue, (v, vertex, self.graph[v][vertex]))

        return mst

    # adj matric looks like [[],[],[]]
    def primsEager(self):
        # Initialize the minimum spanning tree and a set to keep track of visited vertices
        mst = []
        visited = set()

        # Create a list to store the minimum-weight edge to each vertex
        key = [float('inf')] * self.V

        # Create a list to store the parent vertex of each vertex in the MST
        parent = [-1] * self.V

        # Choose a starting vertex (you can choose any vertex, I'll choose 0 here)
        start_vertex = 0
        key[start_vertex] = 0

        while len(visited) < self.V:
            # Find the vertex with the minimum key value that has not been visited yet
            min_key = float('inf')
            min_vertex = None

            for v in range(self.V):
                if key[v] < min_key and v not in visited:
                    min_key = key[v]
                    min_vertex = v

            # If no minimum vertex is found, break the loop
            if min_vertex is None:
                break

            # Mark the vertex as visited
            visited.add(min_vertex)

            # Add the minimum-weight edge to the minimum spanning tree
            if parent[min_vertex] != -1:
                u = min_vertex
                v = parent[u]
                w = self.graph[u][v]
                mst.append((u, v, w))

            # Update key values for adjacent vertices
            for v in range(self.V):
                if self.graph[min_vertex][v] > 0 and v not in visited and self.graph[min_vertex][v] < key[v]:
                    key[v] = self.graph[min_vertex][v]
                    parent[v] = min_vertex  # Update the parent of v

        return mst


class GraphAdjList:
    def __init__(self, vertices, edges, random=False,):
        self.V = vertices
        self.graph = {}

        if random:
            self.graph = generateRandomAdjAdjList(vertices, edges, 10)

        else:
            for i in range(vertices):
                self.graph[i] = []

    def addEdge(self, u, v, w):
        # Add to adjacency representation using dictionaries
        if u not in self.graph:
            self.graph[u] = []
        self.graph[u].append((v, w))

        if v not in self.graph:
            self.graph[v] = []
        self.graph[v].append((u, w))

    # graph structure looks like {0: [(1, 4), (7, 8)], 1: [(0, 4), (2, 8), (7, 11)], 2: [(1, 8), (3, 7), (5, 4), (8, 2)], 3: [(2, 7), (4, 9), (5, 14)], 4: [(3, 9), (5, 10)], 5: [(2, 4), (3, 14), (4, 10), (6, 2)], 6: [(5, 2), (7, 1), (8, 6)], 7: [(0, 8), (1, 11), (6, 1), (8, 7)], 8: [(2, 2), (6, 6), (7, 7)]}

    def getEdges(self):
        edges = []
        for i in self.graph:
            for j in self.graph[i]:
                edges.append((i, j[0], j[1]))
        return edges

    def kruskelForAdjList(self):
        result = []
        i, e = 0, 0

        # Create a list of edges as tuples (u, v, w)
        edges = self.getEdges()

        # Sort the list of edges by weight
        edges.sort(key=lambda item: item[2])

        parent = list(range(self.V))
        rank = [0] * self.V

        while e < self.V - 1:
            if i >= len(edges):
                break
            u, v, w = edges[i]
            i = i + 1
            x = self.find(parent, u)
            y = self.find(parent, v)

            if x != y:
                e = e + 1
                result.append((u, v, w))
                self.union(parent, rank, x, y)

        return result

    def find(self, parent, i):
        if parent[i] == i:
            return i
        return self.find(parent, parent[i])

    def union(self, parent, rank, x, y):
        xroot = self.find(parent, x)
        yroot = self.find(parent, y)

        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot
        else:
            parent[yroot] = xroot
            rank[xroot] += 1

    def primsEager(self):
        # Initialize the minimum spanning tree and a set to keep track of visited vertices
        mst = []
        visited = set()

        # Create a list to store the minimum-weight edge to each vertex
        key = [float('inf')] * self.V

        # Create a list to store the parent vertex of each vertex in the MST
        parent = [-1] * self.V

        # Choose a starting vertex (you can choose any vertex, I'll choose 0 here)
        start_vertex = 0
        key[start_vertex] = 0

        while len(visited) < self.V:
            # Find the vertex with the minimum key value that has not been visited yet
            min_key = float('inf')
            min_vertex = None

            for v in range(self.V):
                if key[v] < min_key and v not in visited:
                    min_key = key[v]
                    min_vertex = v

            # If no minimum vertex is found, break the loop
            if min_vertex is None:
                break

            # Mark the vertex as visited
            visited.add(min_vertex)

            # Add the minimum-weight edge to the minimum spanning tree
            if parent[min_vertex] != -1:
                u = min_vertex
                v = parent[u]
                for neighbor, weight in self.graph[u]:
                    if neighbor == v:
                        w = weight
                        mst.append((u, v, w))

            # Update key values for adjacent vertices
            for neighbor, weight in self.graph[min_vertex]:
                if weight < key[neighbor] and neighbor not in visited:
                    key[neighbor] = weight
                    # Update the parent of neighbor
                    parent[neighbor] = min_vertex

        return mst

    def primsLazy(self):
        # Initialize the minimum spanning tree and a set to keep track of visited vertices
        mst = []
        visited = set()

        # Choose a starting vertex (you can choose any vertex, I'll choose 0 here)
        start_vertex = 0

        # Create a priority queue to store edges with their weights
        edge_queue = []

        # Add all edges connected to the starting vertex to the priority queue
        for neighbor, weight in self.graph[start_vertex]:
            edge_queue.append((start_vertex, neighbor, weight))

        # Mark the starting vertex as visited
        visited.add(start_vertex)

        # Use a heap data structure to efficiently retrieve the minimum-weight edge
        import heapq
        heapq.heapify(edge_queue)

        while edge_queue:
            u, v, w = heapq.heappop(edge_queue)

            if v in visited:
                continue

            mst.append((u, v, w))
            visited.add(v)

            # Add all edges connected to vertex v to the priority queue
            for neighbor, weight in self.graph[v]:
                if neighbor not in visited:
                    heapq.heappush(edge_queue, (v, neighbor, weight))

        return mst
