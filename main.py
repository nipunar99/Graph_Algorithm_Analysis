# main
from graphs import GraphAdjMatrix, GraphAdjList
from utils import evaluate

if __name__ == "__main__":

    # setup sparse graph
    num_nodes_sparse = 5000
    num_edges_sparse = 5000

    # setup dense graph
    num_nodes_dense = 5000
    num_edges_dense = 5000 * (5000 - 1) // 4
    choice = 0

    # get user input and create graphs and do analysis
    while choice != 5:

        if choice == 0:
            print("Enter relevent choice to execute actions:")
            print(
                "1. Analyze sparse graph using adjacency matrix | nodes: 5000, edges: 5000")
            print(
                "2. Analyze sparse graph using adjacency list | nodes: 5000, edges: 5000")
            print(
                f"3. Analyze dense graph using adjacency matrix | nodes: 5000, edges: {(5000 * (5000 - 1) // 4)}")
            print(
                f"4. Analyze dense graph using adjacency list | nodes: 5000, edges: {(5000 * (5000 - 1) // 4)}")
            print("5. Exit")

            choice = int(input("Enter your choice: "))

        elif choice == 1:
            g_sparse_adj_matrix = GraphAdjMatrix(
                num_nodes_sparse, edges=num_edges_sparse, random=True)
            print("\n\nEvaluating sparse graph using adjacency matrix: ")
            print("\nKruskal's algorithm: ")
            evaluate(g_sparse_adj_matrix.kruskelForAdjMatrix)
            print("\nPrim's Eager algorithm: ")
            evaluate(g_sparse_adj_matrix.primsEager)
            print("\nPrim's Lazy algorithm: ")
            evaluate(g_sparse_adj_matrix.primsLazy)
            print("\n\n")
            choice = 0

        elif choice == 2:
            g_sparse_adj_list = GraphAdjList(
                num_nodes_sparse, edges=num_edges_sparse, random=True)
            print("\nEvaluating sparse graph using adjacency list: ")
            print("\nKruskal's algorithm: ")
            evaluate(g_sparse_adj_list.kruskelForAdjList)
            print("\nPrim's Eager algorithm: ")
            evaluate(g_sparse_adj_list.primsEager)
            print("\nPrim's Lazy algorithm: ")
            evaluate(g_sparse_adj_list.primsLazy)
            print("\n\n")
            choice = 0

        elif choice == 3:
            g_dense_adj_matrix = GraphAdjMatrix(
                num_nodes_dense, edges=num_edges_dense, random=True)
            print("\nEvaluating dense graph using adjacency matrix: ")
            print("\nKruskal's algorithm: ")
            evaluate(g_dense_adj_matrix.kruskelForAdjMatrix)
            print("\nPrim's Eager algorithm: ")
            evaluate(g_dense_adj_matrix.primsEager)
            print("\nPrim's Lazy algorithm: ")
            evaluate(g_dense_adj_matrix.primsLazy)
            print("\n\n")
            choice = 0

        elif choice == 4:
            g_dense_adj_list = GraphAdjList(
                num_nodes_dense, edges=num_edges_dense, random=True)
            print("\nEvaluating dense graph using adjacency list: ")
            print("\nKruskal's algorithm: ")
            evaluate(g_dense_adj_list.kruskelForAdjList)
            print("\nPrim's Eager algorithm: ")
            evaluate(g_dense_adj_list.primsEager)
            print("\nPrim's Lazy algorithm: ")
            evaluate(g_dense_adj_list.primsLazy)
            print("\n\n")
            choice = 0

        elif choice == 5:
            exit()

        else:
            print("Invalid choice")

    # print("Enter number of nodes: ")

    # Create a random graphs

    # sparse
    # adjecency matrix
    # g_sparse_adj_matrix = GraphAdjMatrix(
    #     num_nodes_sparse, edges=num_edges_sparse, random=True)

    # # adjecency list
    # g_sparse_adj_list = GraphAdjList(
    #     num_nodes_sparse, edges=num_edges_sparse, random=True)

    # # dense
    # # adjecency matrix
    # g_dense_adj_matrix = GraphAdjMatrix(
    #     num_nodes_dense, edges=num_edges_dense, random=True)

    # # adjecency list
    # g_dense_adj_list = GraphAdjList(
    #     num_nodes_dense, edges=num_edges_dense, random=True)

    # evaluate the performance of algorithms on sparse graph
    # print("Sparse graph:\n")
    # print("Adjacency matrix:")
    # print("Kruskal's algorithm:")
    # evaluate(g_sparse_adj_matrix.kruskelForAdjMatrix)
    # print("Prim's Eager algorithm:")
    # evaluate(g_sparse_adj_matrix.primsEager)
    # print("Prim's Lazy algorithm:")
    # evaluate(g_sparse_adj_matrix.primsLazy)

    # print("\nAdjacency list:")
    # print("Kruskal's algorithm:")
    # evaluate(g_sparse_adj_list.kruskelForAdjList)
    # print("Prim's Eager algorithm:")
    # evaluate(g_sparse_adj_list.primsEager)
    # print("Prim's Lazy algorithm:")
    # evaluate(g_sparse_adj_list.primsLazy)

    # # evaluate the performance of algorithms on dense graph
    # print("\nDense graph:\n")
    # print("Adjacency matrix:")
    # print("Kruskal's algorithm:")
    # evaluate(g_dense_adj_matrix.kruskelForAdjMatrix)
    # print("Prim's Eager algorithm:")
    # evaluate(g_dense_adj_matrix.primsEager)
    # print("Prim's Lazy algorithm:")
    # evaluate(g_dense_adj_matrix.primsLazy)

    # print("\nAdjacency list:")
    # print("Kruskal's algorithm:")
    # evaluate(g_dense_adj_list.kruskelForAdjList)
    # print("Prim's Eager algorithm:")
    # evaluate(g_dense_adj_list.primsEager)
    # print("Prim's Lazy algorithm:")
    # evaluate(g_dense_adj_list.primsLazy)

    print("success")
