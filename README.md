# computer-science


# Binary Search Algorithm

	The array of elements MUST BE SORTED
	Compared to the midpoint to know if we find the element or is in the left side or if the element is in the right side.
	Then repeat the same process with the left side or right side.

	https://www.youtube.com/watch?v=P3YID7liBug

	Time Complexity:

		O(log n)

		Iteration	#Elements

		1			n
		2 			n/2
		3 			n/4
		4 			n/8

		k 			n/2^k

		n * 1	 = 1 			// 1 because at the end we need only 1 element in the array in order to find the target element
			-	
			2^k

		2^k = n

		k = log n

	Recursive Implementation:

		Auxiliary Space: (Log n) recursion call stack space.

		boolean binarySearchRecursive(int[] array, int x, int left, int right) {

			if (left > right) {
				return false;
			}

			int mid = left + ( (right - left) / 2 ); // prevent overflow

			if ( x == array[mid]) {
				return true;
			} else if ( x < array[mid]) {
				binarySearchRecursive(array, x, left, mid - 1);
			} else {
				binarySearchRecursive(array, x, mid + 1, right);
			}
		}

		boolean binarySearchRecursive(int[] array, int x) {
			return binarySearchRecursive(array, x, 0, array.length - 1);
		}

	Iterative Implementation:

		Auxiliary Space: O(1)


		boolean binarySearchIterative(int[] array, int x) {
			int left = 0;
			int right = array.length - 1;
			while (left < right) {
				int mid = left + ( (right - left) / 2 );
				if (x == array[x]) {
					return true;
				} else if ( x < array[mid]) {
					right = mid - 1;
				} else {
					left = mid + 1;
				}
			}
			return false;
		}


# BST: Binary Search Tree

	Binary Search Tree = Ordered Binary Tree
	
	A binary search tree fulfills a specific ordering property.
		-	On any subtree the left node are less than the root node which is less than all the right nodes.
		-	This ordering property makes finding a node VERY VERY fast because we have a pretty good idea of where it would be.
		-	at each operation we chopped off hopefully about half of the nodes and we do that over and over again very very quickly we find the node we're looking for.

	Reference:
		https://www.youtube.com/watch?v=oSWTXtMglKE

	Time Complexity:

		Balanced Binary Tree:
		---------------------

			Insert:		O( log n )
			Find:		O( log n )

		UnBalanced Binary Tree:
		-----------------------

			It looks like a List not a Tree.

			Insert:		O( n )
			Find:		O( n )

	There are algorithms that can ensure that our tree atays balanced, that is that roughly the same number of nodes will be on the left side the subtree and on the right. Those algorithms are complicated.

	Traversing a Tree
	-----------------

		Typically for Binary Search Trees we perform InOrder Traversals because it actually allows the nodes to be printed in order.

		InOrder:

				1. Visit Left
				2. Visit Node
				3. Visit Right

		PreOrder:

				1. Visit Node
				2. Visit Left
				3. Visit Right

		PostOrder:

				1. Visit Left
				2. Visit Right
				3. Visit Node


	Implementation:

		class Node {
			Node left, right;
			int data;

			Node(int data) {
				this.data = data;
			}

			void insert(int value) {
				if (value <= data) {
					if (left == null) {
						left = new Node(value);
					} else {
						left.insert(value);
					}
				} else {
					if (right == null) {
						right = new Node(value);
					} else {
						right.insert(value);
					}
				}
			}

			boolean contains(int value) {
				if (value == data) {
					return true;
				} else if (value < data) {
					if (left == null) {
						return false;
					} else {
						return left.contains(value);
					}
				} else {
					if (right == null) {
						return false;
					} else {
						return right.contains(value);
					}
				}
			}

			public void printInOrder() {
				if (left != null) {
					left.printInOrder();
				}
				System.out.println(data);
				if (right != null) {
					right.printInOrder();
				}
			}
		}

	Example:
		Is This a Binary Search Tree?
		The key is that based on the problem's description is that the BST does not allow duplicates.
		https://github.com/viktorcardona/hackerrank/blob/master/006-bst-is/src/Solution.java


# DFS: Depth First Search

	Depth-first search (DFS) is an algorithm for traversing or searching tree or graph data structures.
	Go forward (in depth)  while there is any such possibility, if not then, backtrack. 
	Backtrack means we have achive a dead end, so move back and choose another path.
	Uses a *** Stack *** data structure
	In the graph we can have cycles, therefore we need to use a boolean visited array. The flag is true if the node was visited.

	Implementation which traverses only the vertices reacheable from a given vertex:

		void DFS(int v) {
			int size = numberOfNodes;
			boolean visited[] = new boolean[size]
			DFSRecursive(v, visited);
		}

		void DFSRecursive(int v, boolean[] visited) {
			visited[v] = true;
			System.out.print( v + " ");

			Iterator<Integer> adjacentNodes = adjacentNodes(v);

			while (adjacentNodes.hasNext()) {
				int nextNode = adjacentNodes.next();
				if (!visited[nextNode]) {
					DFSRecursive(nextNode, visited);
				}
			}
		}

	Implementation which traverses all vertices even when some of them are disconnected:

		void DFS(int v) {
			int numberOfNodes;
			boolean visited[] = new boolean[numberOfNodes]

			for (int i=0; i < numberOfNodes; i++) {
				if (!visited[i]) {
					DFSRecursive(v, visited);
				}
			}
		}

		void DFSRecursive(int v, boolean[] visited) {
			visited[v] = true;
			System.out.print( v + " ");

			Iterator<Integer> adjacentNodes = adjacentNodes(v);

			while (adjacentNodes.hasNext()) {
				int nextNode = adjacentNodes.next();
				if (!visited[nextNode]) {
					DFSRecursive(nextNode, visited);
				}
			}
		}

	References:
		https://en.wikipedia.org/wiki/Depth-first_search
		https://www.youtube.com/watch?v=Y40bRyPQQr0


	Applications:
		- Used to detect cycles in a grapgh. 
		- Grapgh cycles detection are used for producing a minimum spanning tree, which is a subgraph where all nodes are connected
			without cycles with the minimum possible cost. The cost is the weight of each edge.
		- Topological Sorting
		- Finding Strongly Connected Components of a graph: A directed graph is called strongly connected if there is a path from each vertex in the graph to every other vertex.

	Time Complexity:

		O(|V| + |E|)

		V: Vertices
		E: Edges

	Implementation Example:
		https://github.com/viktorcardona/hackerrank/blob/master/004-kruskal/src/dfs/Solution.java
		https://github.com/viktorcardona/hackerrank/blob/master/007-dfs-cciag/src/Solution.java


# BFS: Breadth First Search

	Breadth-first search (BFS) is an algorithm for traversing or searching tree or graph data structures.

	Reference:
		https://www.youtube.com/watch?v=KiCBXu4P-2Y

	BFS: https://upload.wikimedia.org/wikipedia/commons/4/46/Animated_BFS.gif

	Applications:

		- Finding the shortest path between two nodes u and v, with path length measured by number of edges.
		- So it finds the shortest path in a maze
			- A maze is an: 
				- undirected graph 
				- unweighted graph
	
	Pseudocode:
		source: https://www.hackerearth.com/practice/algorithms/graphs/breadth-first-search/tutorial/

		BFS (G, s) //Where G is the graph and s is the source node
      		let Q be queue.
      		Q.enqueue( s ) //Inserting s in queue until all its neighbour vertices are marked.

      		mark s as visited. // it could be used a boolean array

      		while ( Q is not empty)
           		//Removing that vertex from queue,whose neighbour will be visited now
           		v  =  Q.dequeue()

          		//processing all the neighbours of v  
          		for all neighbours w of v in Graph G
               		if w is not visited 
               			Q.enqueue( w )             //Stores w in Q to further visit its neighbour
                        mark w as visited.

	Time complexity:

		O(|V|+|E|)

		Since every vertex and every edge will be explored in the worst case.

	Example:

		https://github.com/viktorcardona/hackerrank/tree/master/002-bfs

# MST: Minimum Spanning Tree

	It is a sub-tree where all nodes are connected with no cycles with the minimum possible cost. The cost is the given weight on each edge.

	Applications:

		Network design

	Algorthms:

		- Kruskal

	Example:
		Using DFS:             https://github.com/viktorcardona/hackerrank/blob/master/004-kruskal/src/dfs/Solution.java
		Using Disjoin DataSet: https://github.com/viktorcardona/hackerrank/blob/master/004-kruskal/src/findunion/Solution.java

# Disjoint Data Structure or Find Union Algorithm

	Simple and strong data structure to keep n sub sets where there are no elements in common this is the intersection between sets are empty.

	It is required only a numeric array 

	It is required the operations:
		- Find
		- Union

	Applications:

		- Use to detect cycles in a graph
		- Use in the Kruskal Algorithm to find a MST: Minimum Spanning Tree


	Example:
		In the Kruskal Algoritm the Disjoin Data Set is used to detect cycles in sub-graphs
		https://github.com/viktorcardona/hackerrank/blob/master/004-kruskal/src/findunion/Solution.java


# Heaps

	Heaps Data Structure

		It is a specialized tree-based data structure that satisfies the heap property.
		We have access in O(1) to the min value if it is a Heap Min.
		We have access in O(1) to the max value if it is a Heap Max.
		Heaps are also crucial in several efficient graph algorithms such as Dijkstra's algorithm.
		Heaps are usually implemented in an array (fixed size or dynamic array), and do not require pointers between elements

	https://en.wikipedia.org/wiki/Heap_(data_structure)
	https://www.youtube.com/watch?v=t0Cq6tVNRBA

	Min HEAP: 
		The root always keeps the minimum value
		So go downs on the tree the values are bigger and bigger
		
		Insertion happens always at the leafs, from left to right.
			Then we have to bubble it up until it gets its spot in the tree.

		Removing the min element.
			Easy to locate the element since it is the root.
			We swap the leaf element and put it in the root then we bubble it down to the next spot so we compare the root with its children

	Implementation Raw:

		Use an array!
		The root contains the min value which is array[0]
		Array:
			Left   Node =  Index * 2 + 1
			Right  Node =  Index * 2 + 2
			Parent Node = (Index - 1) / 2

		class MinHeap {

			int capacity = 10;
			int size = 0;

			int[] items = new int[capacity];

			int getLeftIndex(int parentIndex) { return 2 * parentIndex + 1; }
			int getRightIndex(int parentIndex) { return 2 * parentIndex + 2; }
			int getParentIndex(int childIndex) { return (childIndex - 1) / 2; }

			boolean hasLeftChild(int index) { return getLeftIndex(index) < size; }
			boolean hasRightChild(int index) { return getRightIndex(index) < size; }
			boolean hasParent(int index) { return getParentIndex(index) >= 0; }

			int leftChild(int index) { return items[getLeftIndex(index)]; }
			int rightChild(int index) { return items[getRighttIndex(index)]; }
			int parent(int index) { return items[getParentIndex(index)]; }

			void swap(int index1, int index2) {
				int temp = items[index1];
				items[index1] = items[index2];
				items[index2] = temp;
			}

			void ensureExtraCapacity() {
				if (size == capacity) {
					items = Arrays.copyOf(items, capacity * 2);
					capacity *= 2;
				}
			}

			
			int peek() {
				if (size == 0) throw new IllegalStateException();
				return items[0];
			}

			int poll() {
				if (size == 0) throw new IllegalStateException();
				int item = items[0];
				items[0] = item[size - 1]
				size--;
				heapifyDown();
				return item;
			}

			void add(int item) {
				ensureExtraCapacity();
				items[size] = item;
				size++;
				heapifyUp();
			}

			void heapifyUp() {
				int index = size - 1;
				while ( hasParent(index) && parent(index) > items[index]) {
					swap(index, getParentIndex(index));
					index = getParentIndex(index);
				}
			}

			void heapifyDown() {
				int indez = 0;
				while (hasLeftChild(index)) {
					int smallerChildIndex = getLeftChildIndex(index);
					if (hasRightIndex(index) && rightChild(index) < leftChild(index))) {
						smallerChildIndex = getRightChildIndex(index);
					}

					if (items[index] < items[smallerChildIndex]) {
						break;
					} else {
						swap(index, smallerChildIndex);
					}

					index = smallerChildIndex;
				}
			}

		}



# Priority Queue

	Data Structure commomly implemented using Heaps

	https://en.wikipedia.org/wiki/Priority_queue

	Implementation in Java: https://docs.oracle.com/javase/8/docs/api/java/util/PriorityQueue.html

	Applications:

		-	Bandwidth Management
		-	Dijkstra's algorithm: When the graph is stored in the form of adjacency list or matrix, priority queue can be used to extract minimum efficiently when implementing Dijkstra's algorithm,
		-	Huffman coding: Huffman coding requires one to repeatedly obtain the two lowest-frequency trees. A priority queue is one method of doing this.



	Example:

		Priority Heaps: Using Java
		https://github.com/viktorcardona/hackerrank/blob/master/005-heaps/src/Solution.java


# Fibonacci Heap 

	Fibonacci heap is a data structure for priority queue operations.
	https://en.wikipedia.org/wiki/Fibonacci_heap

	Fibonacci Heap is a collection of trees satisfying the minimum-heap property, that is, the key of a child is always greater than or equal to the key of the parent. This implies that the minimum key is always at the root of one of the trees. Compared with binomial heaps, the structure of a Fibonacci heap is more flexible.

	Time complexity:
		Insert			Θ(1)
		Find-min		Θ(1)
		Delete-min		O(log n)
		Decrease-key	Θ(1)
		Merge			Θ(1)


# Trie
	
	A trie, also called digital tree, radix tree or prefix tree is a kind of search tree.
	Where the keys are usually strings
	Unlike a binary search tree, no node in the tree stores the key associated with that node; instead, its position in the tree defines the key with which it is associated
	All the descendants of a node have a common prefix of the string associated with that node, and the root is associated with the empty string
	Keys tend to be associated with leaves.
	For the space-optimized presentation of prefix tree, see compact prefix tree.
	Every node of Trie consists of multiple branches. Each branch represents a possible character of keys.
	Mark the last node of every key as end of word node.

	Reference:
		https://en.wikipedia.org/wiki/Trie
		https://www.geeksforgeeks.org/trie-insert-and-search

	Applications:
	 	- A common application of a trie is storing a predictive text or autocomplete dictionary, such as found on a mobile telephone

	Time Complexity:

	 	- O(key_length)

	Space Complexity:

	 	- O(ALPHABET_SIZE * key_length * N) where N is number of keys in Trie.
	
	- Insert and Find run in O(n) time, where n is the length of the key. However the penalty is on Trie storage requirements.

	A simple structure to represent nodes of English alphabet can be as following:
	 	struct TrieNode
		{
		     struct TrieNode *children[ALPHABET_SIZE];
		     // isEndOfWord is true if the node
		     // represents end of a word
		     bool isEndOfWord;
		};
		
		- Every character of input key is inserted as an individual Trie node.
		- The children is an array of pointers (or references) to next level trie nodes.
		- The key length determines Trie depth.


