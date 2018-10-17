# computer-science


# BFS: Breadth First Search

	Applications:

		- Find the shortest path in a maze
			- A maze is an: 
				- undirected graph 
				- unweighted graph
	
	Pseudocode:

		- Use a Queue Data Structure



	The time complexity can be expressed as {\displaystyle O(|V|+|E|)} O(|V|+|E|), since every vertex and every edge will be explored in the worst case. {\displaystyle |V|} |V| is the number of vertices and {\displaystyle |E|} |E| is the number of edges in the graph.

# MST: Minimum Spanning Tree

	It is a sub-tree where all nodes are connected with no cycles with the minimum possible cost

	Applications:

		Network design

	Algorthms:

		- Kruskal

# Disjoint Data Structure or Find Union Algorithm

	Simple and strong data structure to keep n sub sets where there are no elements in common this is the intersection between sets are empty.

	It is required only a numeric array 

	It is required the operations:
		- Find
		- Union

	Applications:

		- Use to detect cycles in a graph
		- Use in the Kruskal Algorithm to find a MST: Minimum Spanning Tree
