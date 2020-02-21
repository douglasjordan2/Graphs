"""
Simple graph implementation
"""
from util import Stack, Queue  # These may come in handy

class Graph:

    """Represent a graph as a dictionary of vertices mapping labels to edges."""
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex_id):
        """
        Add a vertex to the graph.
        """
        self.vertices[vertex_id] = set()

    def add_edge(self, v1, v2):
        """
        Add a directed edge to the graph.
        """
        if v1 in self.vertices and v2 in self.vertices:
            self.vertices[v1].add(v2)

    def get_neighbors(self, vertex_id):
        """
        Get all neighbors (edges) of a vertex.
        """
        return self.vertices[vertex_id]

    def bft(self, starting_vertex):
        """
        Print each vertex in breadth-first order
        beginning from starting_vertex.
        """
        # make a queue
        queue = Queue()
        # make a set for visited nodes
        visited = set()

        # put our starting node in line
        queue.enqueue(starting_vertex)

        # if our queue is not empty, we have more nodes to visit
        while queue.size() > 0:
            # get next node out of line
            current_node = queue.dequeue()

            # check if it has been visited
            if current_node not in visited:
                # if not, mark as visited
                visited.add(current_node)
                print(current_node)

                # get its neighbors
                edges = self.get_neighbors(current_node)

                # put them in queue to be visited
                for edge in edges:
                    queue.enqueue(edge)

    def dft(self, starting_vertex):
        """
        Print each vertex in depth-first order
        beginning from starting_vertex.
        """
        
        # make a stack
        stack = Stack()

        # make a set for visited nodes
        visited = set()

        # put our starting node at the top of the stack
        stack.push(starting_vertex)

        # if our stack is not empty, we have more nodes to visit
        while stack.size() > 0:
            # get the next node off the top of our stack
            current_node = stack.pop()

            # check if it has been visited
            if current_node not in visited:
                # if not, mark as visited
                visited.add(current_node)
                print(current_node)

                # get its neighbors
                edges = self.get_neighbors(current_node)

                # stack them on the stack to be visited
                for edge in edges:
                    stack.push(edge)

    def dft_recursive(self, vertex, visited = set()):
        """
        Print each vertex in depth-first order
        beginning from starting_vertex.

        This should be done using recursion.
        """
        visited.add(vertex)

        edges = self.get_neighbors(vertex)

        if len(edges) == 0:
            return
        else:
            for edge in edges:
                if edge not in edges:
                    self.dft_recursive(edge, visited)
                else:
                    return

    def factorial(self, number):
        if number == 1:
            return number
        else:
            return number * self.factorial(number - 1)
        

    def bfs(self, starting_vertex, destination_vertex):
        """
        Return a list containing the shortest path from
        starting_vertex to destination_vertex in
        breath-first order.
        """
        # make a queue
        queue = Queue()

        # make a set for visited
        visited = set()

        # enqueue a path to the starting vortex
        queue.enqueue([starting_vertex])

        while queue.size() > 0:
            current_path = queue.dequeue()
            # current_node is the last thing in the path
            current_node = current_path[-1]
            if current_node == destination_vertex:
                return current_path
            else:
                if current_node not in visited:
                    visited.add(current_node)
                    edges = self.get_neighbors(current_node)

                    for edge in edges:
                        path_copy = list(current_path)
                        path_copy.append(edge)
                        queue.enqueue(path_copy)

    def dfs(self, starting_vertex, destination_vertex):
        """
        Return a list containing a path from
        starting_vertex to destination_vertex in
        depth-first order.
        """
        stack = Stack()
        visited = set()

        stack.push([starting_vertex])

        while stack.size() > 0:
            current_path = stack.pop()
            current_node = current_path[-1]

            if current_node == destination_vertex:
                return current_path
            else:
                if current_node not in visited:
                    visited.add(current_node)
                    edges = self.get_neighbors(current_node)

                    for edge in edges:
                        path_copy = list(current_path)
                        path_copy.append(edge)
                        stack.push(path_copy)

    def dfs_recursive(self, starting_vertex, destination_vertex, visited=set(), path=[]):
        """
        Return a list containing a path from
        starting_vertex to destination_vertex in
        depth-first order.

        This should be done using recursion.
        """
        visited.add(starting_vertex)
        path = path + [starting_vertex]

        if starting_vertex == destination_vertex:
            return path
        
        edges = self.get_neighbors(starting_vertex)
        for edge in edges:
            if edge not in visited:
                new_path = self.dfs_recursive(edge, destination_vertex, visited, path)
                if new_path:
                    return new_path

if __name__ == '__main__':
    graph = Graph()  # Instantiate your graph
    # https://github.com/LambdaSchool/Graphs/blob/master/objectives/breadth-first-search/img/bfs-visit-order.png
    graph.add_vertex(1)
    graph.add_vertex(2)
    graph.add_vertex(3)
    graph.add_vertex(4)
    graph.add_vertex(5)
    graph.add_vertex(6)
    graph.add_vertex(7)
    graph.add_edge(5, 3)
    graph.add_edge(6, 3)
    graph.add_edge(7, 1)
    graph.add_edge(4, 7)
    graph.add_edge(1, 2)
    graph.add_edge(7, 6)
    graph.add_edge(2, 4)
    graph.add_edge(3, 5)
    graph.add_edge(2, 3)
    graph.add_edge(4, 6)

    '''
    Should print:
        {1: {2}, 2: {3, 4}, 3: {5}, 4: {6, 7}, 5: {3}, 6: {3}, 7: {1, 6}}
    '''
    print(graph.vertices)

    '''
    Valid BFT paths:
        1, 2, 3, 4, 5, 6, 7
        1, 2, 3, 4, 5, 7, 6
        1, 2, 3, 4, 6, 7, 5
        1, 2, 3, 4, 6, 5, 7
        1, 2, 3, 4, 7, 6, 5
        1, 2, 3, 4, 7, 5, 6
        1, 2, 4, 3, 5, 6, 7
        1, 2, 4, 3, 5, 7, 6
        1, 2, 4, 3, 6, 7, 5
        1, 2, 4, 3, 6, 5, 7
        1, 2, 4, 3, 7, 6, 5
        1, 2, 4, 3, 7, 5, 6
    '''
    graph.bft(1)

    '''
    Valid DFT paths:
        1, 2, 3, 5, 4, 6, 7
        1, 2, 3, 5, 4, 7, 6
        1, 2, 4, 7, 6, 3, 5
        1, 2, 4, 6, 3, 5, 7
    '''
    graph.dft(1)
    graph.dft_recursive(1)

    '''
    Valid BFS path:
        [1, 2, 4, 6]
    '''
    print(graph.bfs(1, 6))

    '''
    Valid DFS paths:
        [1, 2, 4, 6]
        [1, 2, 4, 7, 6]
    '''
    print(graph.dfs(1, 6))
    print(graph.dfs_recursive(1, 6))
