# Python3 program to implement traveling salesman
# problem using naive approach.
from sys import maxsize
from itertools import permutations
V = 8

# implementation of traveling Salesman Problem
def travellingSalesmanProblem(graph, s):

	# store all vertex apart from source vertex
	vertex = []
	for i in range(V):
		if i != s:
			vertex.append(i)

	# store minimum weight Hamiltonian Cycle
	min_path = maxsize
	templist = []
	pathweightlist = []
	shortestpathlist = []
	minlist = []
	next_permutation=permutations(vertex)
	for i in next_permutation:

		# store current Path weight(cost)
		current_pathweight = 0

		# compute current path weight
		k = s
		for j in i:
			current_pathweight += graph[k][j]
			templist.append((k,j))
			shortestpathlist.append((k,j))
			k = j
		current_pathweight += graph[k][s]
		templist.append((k,s))
		shortestpathlist.append((k,s))
		templist.append(current_pathweight)
		for index, i in enumerate(templist):
			if((index+1) % 9 == 0):
				pathweightlist.append(i)
		templist.clear()
		minindex = pathweightlist.index(min(pathweightlist))
		# update minimum
		min_path = min(min_path, current_pathweight)
	for i in shortestpathlist[minindex*8:minindex*8+8]:
		minlist.append(i)
	return round(min_path, 1), minlist


# Driver Code
if __name__ == "__main__":

	# matrix representation of graph
	graph = [[0, 14.6, 3.4, 1.5, 3.4, 2.6, 2.6, 2.5],
	[14.6, 0, 14.2, 15.2, 14.4, 13.2, 16, 16],
	[3.4, 14.2, 0, 2.2, 6.7, 5.4, 2.7, 3],
	[1.5, 15.2, 2.2, 0, 4.3, 3.6, 2, 1.9],
	[3.4, 14.4, 6.7, 4.3, 0, 1.3, 6.4, 6.7],
	[2.6, 13.2, 5.4, 3.6, 1.3, 0, 5.3, 5.5],
	[2.6, 16, 2.7, 2, 6.4, 5.3, 0, 0.5],
	[2.5, 16, 3, 1.9, 6.7, 5.5, 0.5, 0]]
	s = 0
	minimumvalue, shortestpath = travellingSalesmanProblem(graph, s)
	print("Shortest distance with full tour is " + str(minimumvalue))
	print("Shortest path is " + str(shortestpath))
