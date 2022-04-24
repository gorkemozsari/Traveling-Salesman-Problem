from asyncio.windows_events import NULL
import numpy as np
from queue import *


def print_menu():
        print(
        "\nTraveling Salesman Problem:\n"
        "[1] Construct a shortest path graph\n"
        "[2] TSP Solution with BSF and UCS\n"
        "[3] Exit"
        )

def returnGrid(filename):

    with open(filename, "r") as file:
        grid=file.read().split("\n")
        #print(grid)

        grid = np.array(grid)
        return grid

def findManhattanDistance (i, j):
    return abs(i[0] - j[0]) + abs(i[1] - j[1])

#Finds the coordinats of the given Character in given Grid, and returns it in tuple format
def findNode(char, grid):
    m = len(grid)
    n = len(grid[0])
    #print(m)
    #print(n)
    for mi in range(m):
        for ni in range(n):
            if char == grid[mi][ni]:
                return (mi,ni)

#Takes the start Character, goal Character and grid, and prints the shortest possible path
def search(grid, startNode, goalNode):

    start = findNode(startNode, grid)
    goal = findNode(goalNode, grid)

    frontier = PriorityQueue()
    frontier.put(start, 0)
    #came_from = dict()
    cost_so_far = dict()
    #came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal: #to prevent infinite loop
            break
    
        for next in returnNeighbours(current):
            x = next[0]
            y = next[1]
            if not grid[x][y] == "*": #Checking walls
                new_cost = cost_so_far[current] + 1 #graph.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + findManhattanDistance(goal, next)
                    frontier.put(next, priority)
                    #came_from[next] = current

    #print("Path is ")
    #print(came_from)
    #print("cost so far")
    #print(cost_so_far)
    #print("cost is")
    #print(cost_so_far[current])
    print(startNode + "," + goalNode + "," + str(cost_so_far[current]))
    return [startNode, goalNode, cost_so_far[current]]
def returnNeighbours(node):
    return [(node[0]-1, node[1]), (node[0], node[1]-1), (node[0], node[1]+1), (node[0]+1, node[1])]


def shortestPathGraph(grid):

    ab = search(grid, 'A', 'B')
    ac = search(grid, 'A', 'C')
    ad = search(grid, 'A', 'D')
    bc = search(grid, 'B', 'C')
    bd = search(grid, 'B', 'D')
    cd = search(grid, 'C', 'D')

    graph = {}
    graph = addVertex(graph, 'A')
    graph = addVertex(graph, 'B')
    graph = addVertex(graph, 'C')
    graph = addVertex(graph, 'D')

    graph = addEdge(graph, ab[0], ab[1], ab[2]) # A -> B
    graph = addEdge(graph, ab[1], ab[0], ab[2]) # B -> A
    graph = addEdge(graph, ac[0], ac[1], ac[2]) # A -> C
    graph = addEdge(graph, ac[1], ac[0], ac[2]) # C -> A
    graph = addEdge(graph, ad[0], ad[1], ad[2]) # A -> D
    graph = addEdge(graph, ad[1], ad[0], ad[2]) # D -> A
    graph = addEdge(graph, bc[0], bc[1], bc[2]) # B -> C
    graph = addEdge(graph, bc[1], bc[0], bc[2]) # C -> B
    graph = addEdge(graph, bd[0], bd[1], bd[2]) # B -> D
    graph = addEdge(graph, bd[1], bd[0], bd[2]) # D -> B
    graph = addEdge(graph, cd[0], cd[1], cd[2]) # C -> D
    graph = addEdge(graph, cd[1], cd[0], cd[2]) # D -> C
    
    return graph


def addVertex(graph, v):
    graph[v] = []
    return graph

def addEdge(graph, v1, v2, e):
    temp = [v2, e]
    graph[v1].append(temp)
    return graph

def printGraph(graph):
  for vertex in graph:
    for edges in graph[vertex]:
      print(vertex, " -> ", edges[0], " edge weight: ", edges[1])


def bfs(graph, node):
    visited = []
    #visited.append(node)
    totalCost = 0
    lastVertex = NULL
    for vertex in graph:
        edgeCounter = 0
        for edges in graph[vertex]:
            if(edges[0]) not in visited and vertex not in visited:
                edgeCounter += 1
                visited.append(vertex)
                if(edgeCounter == len(graph) -1):
                    visited.append(edges[0])
                    lastVertex = edges[0]
                #print(visited)
                totalCost += edges[1]
                #print(totalCost)
            else:
                edgeCounter += 1

    if (lastVertex != NULL):
        for edges in graph[lastVertex]:
            if edges[0] == node:
                totalCost += edges[1]
                visited.append(node)

    tourPath = printChange(visited)
    
    print("Algorithm Used: BFS\n")
    print(tourPath)
    print("\nTotal Tour Cost: " + str(totalCost))
    return [tourPath, totalCost]

def ucs (graph, node):
    visited = []
    visited.append(node)
    totalCost = 0
    nodeCount = 0
    nextVertex = NULL
    for vertex in graph:
        if nextVertex == NULL: # For selecting first vertex like base case of recursive
            edgeCosts = []
            edgeNo = 0
            for edges in graph[vertex]:
                if edges[0] not in visited: # First check the cost of unvisited nodes
                    edgeCosts.append(edges[1])
            #print(edgeCosts)
            for edges in graph[vertex]:
                if edges[0] not in visited:
                    if edgeNo == edgeCosts.index(min(edgeCosts)): # Then find th eindex of minimum cost 
                        visited.append(edges[0]) # Then add the node that is connected to that cost
                        totalCost += edges[1]
                        nodeCount += 1
                        nextVertex = edges[0]
                    else:
                        edgeNo += 1
            #print(visited)
            #print(totalCost)
        elif nodeCount < 3: # Selecting next vertices
            edgeCosts = []
            edgeNo = 0
            #print(graph[nextVertex])
            #print(visited)
            for edges in graph[nextVertex]:
                if edges[0] not in visited: # First check the cost of unvisited nodes
                    edgeCosts.append(edges[1])
            #print(edgeCosts)
            for edges in graph[nextVertex]:
                if edges[0] not in visited:
                    if edgeNo == edgeCosts.index(min(edgeCosts)): # Then find th eindex of minimum cost 
                        visited.append(edges[0]) # Then add the node that is connected to that cost
                        totalCost += edges[1]
                        nodeCount += 1
                        nextVertex = edges[0]
                    else:
                        edgeNo += 1
            #print(visited)
            #print(totalCost)
        else: # For returning back to starting node
            for edges in graph[nextVertex]:
                if edges[0] == node:
                    visited.append(edges[0]) # Then add the node that is connected to that cost
                    totalCost += edges[1]
            #print(visited)
            #print(totalCost)
    tourPath = printChange(visited)
    print("Algorithm Used: UCS\n")
    print(tourPath)
    print("\nTotal Tour Cost: " + str(totalCost))
    return [tourPath, totalCost]


def printChange(visited): # Just to match the printing format like in the pdf (Example: From ['A', 'D', 'C', 'B', 'A'] to A-D-C-B-A)
    tourPath = ""
    for x in range(len(visited)):
        if(x != len(visited)-1):
            tourPath += visited[x] + "-"
        else:
            tourPath += visited[x] # To stop printing last line(-)
    return tourPath

def tsp(graph, node):
    bfsvalues = bfs(graph, node)
    bfstime = "O(" + str(len(graph) + len(graph)**2) + ")"  # Because we used adjacency list for Breath-First Search, Time complexity is O(V + E)
    print()                                                 # V is vertices and E is edges. Also because this is a complete graph, edge number is the square of vertice number
    ucsvalues = ucs(graph, node)
    ucstime = "O(" + str(4**(1 + (4/1))) + ")"                # O(b1 + [C*/ε]) b is branching factor b = 4 in our case. C* is cost to optimal solution which is 4,  
    print("\nStatistics:")                      # ε is each step to get closer to the goal node 1 in our case
    print("\tNodes\t\tTime\t\tCost")
    print("BFS" + "\t" + bfsvalues[0] + "\t" + bfstime + "\t\t" + str(bfsvalues[1]))
    print("UCS" + "\t" + ucsvalues[0] + "\t" + ucstime + "\t" + str(ucsvalues[1]))

grid = returnGrid("input.txt")
graph = NULL
while True:
    print_menu()
    option = int(input("Selection: "))
    if option == 1:
        graph = shortestPathGraph(grid)
    elif option == 2:
        if(graph == NULL):
            print("Please contruct the shortest path graph first\n")
        else:
            #printGraph(graph)
            tsp(graph, 'A')
    elif option == 3:
        exit()
    else:
        print("Invalid option. Please enter a number between 1 and 3.")