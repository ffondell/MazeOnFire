import numpy as np
from numpy import zeros
import random
from matplotlib import pyplot as plt

class Node:#easier to use nodes based off location in grid and a value to show whether it is a blocked square or not
    def __init__(self, row, col, val):
        self.row = row
        self.col = col
        self.val = val


def strategy1(grid, start, goal, q):

    shortestPath = getShortestPath(grid, start, goal, len(grid))
    i = len(shortestPath)-1
    print(grid)
    while(i!=0):
        if(grid[shortestPath[i].row][shortestPath[i].col]==2):
            return False
        grid[i][j]=9
        grid = advanceFire(grid,q)
        i-=1
    print(grid)
    return True#strategy that calculates the shortest path once and travels along it until goal or fire


def strategy2(grid, start, goal, q):
    shortestPath = getShortestPath(grid, start, goal, len(grid))
    run = True
    i = len(shortestPath)-1


    while(i!=0):
        #print("\ni: "+str(i))
        #print("list length: "+str(len(shortestPath)))
        grid = advanceFire(grid, q)
        #print("current node: "+str(shortestPath[i].row)+", "+str(shortestPath[i].col))
        if(grid[goal.row][goal.col]==2):
            return False
        if(grid[shortestPath[i].row][shortestPath[i].col]==2):
            return False
        shortestPath = getShortestPath(replaceTwos(grid), shortestPath[i-1], shortestPath[0], len(grid))
        i = len(shortestPath)-1

    print(grid)
    return True#strategy that calcuates the shortest path after every step

def frankStrategy(grid, start, goal, q):#strategy that calcualtes the shortest path by look 3 steps ahead at each step
    shortestPath = getShortestPath(grid, start, goal, len(grid))
    i = len(shortestPath)-1
    while(i!=0):
        print("here")
        print("i: "+str(i))
        booly1 = False
        copyGrid = grid
        copyGrid = advanceFire(copyGrid, q)
        copyGrid = advanceFire(copyGrid, q)
        copyGrid = advanceFire(copyGrid, q)
        booly1, throwaway = bfs(replaceTwos(copyGrid), Node(shortestPath[i].row, shortestPath[i].col, 0) , goal, len(grid))
        if(booly1):
            shortestPath = getShortestPath(copyGrid, shortestPath[i-1], shortestPath[0], len(grid))
            copyGrid = None
        i = len(shortestPath)-1
        grid = advanceFire(grid, q)
        if(grid[goal.row][goal.col]==2):
            return False
        if(grid[shortestPath[i].row][shortestPath[i].col]==2):
            return False
    return True


def replaceTwos(grid):
    copyGrid = grid.copy()
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if(grid[i][j]==2):
                copyGrid[i][j]=1
    return copyGrid#replaces the twos with ones on my grid so I wouldnt have to rewrite my bfs to avoid squares with a ONE and a TWO

def reconstructPath(path, grid):
    path.reverse()
    neighborFound = False
    for i in range(len(path)):
        #print("path length"+str(len(path)))
        #print(str(i))
        #print("here")
        if(i<len(path)-1):
            neighborFound = False
            #print("here1")
            while(not(neighborFound)):
                #print("\nNode at: "+str(i))
                #print("\npath length: "+str(len(path)))
                if(i<len(path)-1):
                    if(not(inVisited(generateKids(path[i], len(grid), grid, [], []), path[i+1]))):
                        path.remove(path[i+1])
                    else:
                        #print("here3")
                        neighborFound = True
                else:
                    neighborFound = True

    return path#takes the fringe from bfs and converts it into the shortest path


def advanceFire(grid, q):#advance fire one step based on number of neighbors already on fire
    copyGrid = grid.copy()
    kids = []
    for i in range(len(grid)):

        for j in range(len(grid[i])):
            if((grid[i][j]!=1)and(grid[i][j]!=2)):
                kids = generateKids(Node(i, j, 0), len(grid), grid, [], [])
                k = 0
                #printNodes(kids)
                for x in range(len(kids)):
                    if(grid[kids[x].row][kids[x].col]==2):
                        #rint("here3")
                        k+=1
                #print("k"+str(k))
                prob = 100*(1-((1-q)**k))
                #print("\nprob: "+str(prob))
                if(random.randint(0,100)<prob):
                    #print("here1")
                    copyGrid[i][j]=2


    return copyGrid


def dfs(grid, start, goal, dim):
    currentNode = Node
    fringe = []
    visited = []
    fringe.append(start)
    count = 0

    while((len(fringe)!=0)and(count!=10)):
        #count+=1
        #print("fringe: ")
        #printNodes(fringe)
        currentNode = fringe.pop(len(fringe)-1)
        #print("here1")

        if((currentNode.row==goal.row) and (currentNode.col==goal.col)):
            return True
        else:
            if(not(currentNode in visited)):#make sure to check if kids r in visited when generated
                fringe.extend(generateKids(currentNode, dim, grid, visited, fringe))

                visited.insert(0, currentNode)
                #print("visited: ")
                #printNodes(visited)

                #print("here3")
    #printNodes(visited)
    #print(inVisited(visited, start
    return False#depth first search


def getShortestPath(grid, start, goal, dim):
    path = []
    currentNode = Node
    fringe = []
    visited = []
    fringe.append(start)
    count = 0
    nodesVisited = 0

    while((len(fringe)!=0)and(count!=10)):
        #count+=1
        #print("fringe: ")
        #printNodes(fringe)
        currentNode = fringe.pop(0)

        nodesVisited+=1
        #print("here1")

        if((currentNode.row==goal.row) and (currentNode.col==goal.col)):
            path.append(currentNode)
            #printNodes(path)
            #print("\n -------- \n")
            #printNodes(reconstructPath(path, grid))
            return reconstructPath(path, grid)
        else:
            if(not(currentNode in visited)):#make sure to check if kids r in visited when generated
                fringe.extend(generateKids(currentNode, dim, grid, visited, fringe))
                path.append(currentNode)
                visited.insert(0, currentNode)
                #print("visited: ")
                #printNodes(visited)

                #print("here3")
    #printNodes(visited)
    #print(inVisited(visited, start))
    return reconstructPath(path, grid)#renaming of bfs, same as bfs put returns the shortest path, only here so I can run either depending on the infromation I need


def bfs(grid, start, goal, dim):

    path = []
    currentNode = Node
    fringe = []
    visited = []
    fringe.append(start)
    count = 0
    nodesVisited = 0

    while((len(fringe)!=0)and(count!=10)):
        #count+=1
        #print("fringe: ")
        #printNodes(fringe)
        currentNode = fringe.pop(0)

        nodesVisited+=1
        #print("here1")

        if((currentNode.row==goal.row) and (currentNode.col==goal.col)):
            path.append(currentNode)
            #printNodes(path)
            #print("\n -------- \n")
            #printNodes(reconstructPath(path, grid))
            return True, nodesVisited
        else:
            if(not(currentNode in visited)):#make sure to check if kids r in visited when generated
                fringe.extend(generateKids(currentNode, dim, grid, visited, fringe))
                path.append(currentNode)
                visited.insert(0, currentNode)
                #print("visited: ")
                #printNodes(visited)

                #print("here3")
    #printNodes(visited)
    #print(inVisited(visited, start))
    return False, nodesVisited#breadth first search

def aStar(grid, start, goal, dim):
    currentNode = Node
    fringe = []
    visited = []
    fringe.append(start)
    count = 0
    nodesVisited = 0

    while((len(fringe)!=0)and(count!=10)):
        #count+=1
        #print("fringe: ")
        #printNodes(fringe)
        currentNode = fringe.pop(getBestNode(fringe, goal))
        nodesVisited+=1
        #print("here1")

        if((currentNode.row==goal.row) and (currentNode.col==goal.col)):
            return True, nodesVisited
        else:
            if(not(currentNode in visited)):#make sure to check if kids r in visited when generated
                fringe.extend(generateKids(currentNode, dim, grid, visited, fringe))

                visited.insert(0, currentNode)
                #print("visited: ")
                #printNodes(visited)

                #print("here3")
    #printNodes(visited)
    #print(inVisited(visited, start))

    return False, nodesVisited#algorithm that uses heursitic to choose fringe nodes, heursitc based off how close the node is to the goal node

def getBestNode(fringe, goal):#checks fringe and chooses node with lowest score(best heuristic)
    indexOfBest = 0
    ideal = goal.row + goal.col
    best = fringe[0]
    for i in range(len(fringe)):
        if(not(i==0)):
            if(abs(ideal - (fringe[i].row + fringe[i].col))<abs(ideal - (best.row + best.col))):
                best = fringe[i]
                indexOfBest = i

    return indexOfBest

def printNodes(nodes):#prints all nodes by coordinates for debug purposes
    for i in range(len(nodes)):
        print("\n"+str(nodes[i].row)+", "+str(nodes[i].col))


def generateKids(node, dim, grid, visited, fringe):#make sure to check value of node, dont add to kids if its blocked
    kids = []
    #print("here2")
    if(node.row-1>=0):#check above Node
        if(not(grid[node.row-1][node.col]==1)):
            toAdd = Node(node.row-1, node.col, 0)
            if((not(inVisited(visited, toAdd)))and(not(inVisited(fringe, toAdd)))):
                #print("here4")
                kids.append(toAdd)
    if(node.col+1<dim):#check right Node
        if(not(grid[node.row][node.col+1]==1)):
            toAdd = Node(node.row, node.col+1, 0)
            if((not(inVisited(visited, toAdd)))and(not(inVisited(fringe, toAdd)))):
                #print("here5")
                kids.append(toAdd)
    if(node.row+1<dim):#check down Node
        if(not(grid[node.row+1][node.col]==1)):
            toAdd = Node(node.row+1, node.col, 0)
            if((not(inVisited(visited, toAdd)))and(not(inVisited(fringe, toAdd)))):
                #print("here6")
                kids.append(toAdd)
    if(node.col-1>=0):#check left Node
        if(not(grid[node.row][node.col-1]==1)):
            toAdd = Node(node.row, node.col-1, 0)
            if((not(inVisited(visited, toAdd)))and(not(inVisited(fringe, toAdd)))):
                #print("here7")
                kids.append(toAdd)
    return kids

def inVisited(nodes, node):#check to see if a node exists in an array
    for i in range(len(nodes)):
        if((node.row==nodes[i].row) and (node.col==nodes[i].col)):
            return True
    return False


def generateGrid(dim, p):#generates grid with given dimension dim and obstacle density d
    grid = zeros([dim,dim])
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if(random.randint(0,100)<p):
                randomVal = 1
            else:
                randomVal = 0
            if(not((i==0 and j==0)or(i==dim-1 and j==dim-1))):
                grid[i][j] = randomVal
    return grid

def setOnFire(grid):#sets a random node on fire to start
    x=0
    y=0
    run = True
    while(run):
        x = random.randint(0,len(grid)-1)
        y = random.randint(0,len(grid)-1)
        if(grid[x][y]==0):
            grid[x][y]=2
            run = False
    return grid, Node(x,y,0)

def plotStrategy2(dim, start, goal):
    run = True
    for q in range(0,100):
        q = float(q)/100.0
        trueCount = 0
        for d in range(1,10):
            run = True
            grid = None

            while(run):
                grid = generateGrid(dim, 30)
                grid, fireNode = setOnFire(grid)
                booly1, throwaway = bfs(grid, Node(0,0,0), Node(dim-1,dim-1,0), dim)
                booly2, throwaway = bfs(grid, Node(0,0,0), Node(fireNode.row, fireNode.col ,0), dim)
                if(booly1 and booly2):
                    run = False
            if(strategy2(grid, start, goal, q)):
                #print("trueCount: "+str(trueCount))
                trueCount+=1
        plt.plot(float(trueCount)/10.0, q, "ob")


    plt.title("avg successes vs flammability q")
    plt.xlabel("avg successes")
    plt.ylabel("flammability q")
    plt.show()#graphs success vs flammability of strategy 1

def plotStrategy1(dim, start, goal):
    run = True
    for q in range(0,100):
        q = float(q)/100.0
        trueCount = 0
        for d in range(1,10):
            run = True
            grid = None

            while(run):
                grid = generateGrid(dim, 30)
                grid, fireNode = setOnFire(grid)
                booly1, throwaway = bfs(grid, Node(0,0,0), Node(dim-1,dim-1,0), dim)
                booly2, throwaway = bfs(grid, Node(0,0,0), Node(fireNode.row, fireNode.col ,0), dim)
                if(booly1 and booly2):
                    run = False
            if(strategy1(grid, start, goal, q)):
                #print("trueCount: "+str(trueCount))
                trueCount+=1
        plt.plot(float(trueCount)/10.0, q, "ob")


    plt.title("avg successes vs flammability q")
    plt.xlabel("avg successes")
    plt.ylabel("flammability q")
    plt.show()#graphs success vs flammability of strategy 2

def plotFrankStrategy(dim, start, goal):
    run = True
    for q in range(0,100):
        q = float(q)/100.0
        trueCount = 0
        for d in range(1,10):
            run = True
            grid = None

            while(run):
                grid = generateGrid(dim, 30)
                grid, fireNode = setOnFire(grid)
                booly1, throwaway = bfs(grid, Node(0,0,0), Node(dim-1,dim-1,0), dim)
                booly2, throwaway = bfs(grid, Node(0,0,0), Node(fireNode.row, fireNode.col ,0), dim)
                if(booly1 and booly2):
                    run = False
            if(frankStrategy(grid, start, goal, q)):
                #print("trueCount: "+str(trueCount))
                trueCount+=1
        print("there")
        plt.plot(float(trueCount)/10.0, q, "ob")


    plt.title("avg successes vs flammability q")
    plt.xlabel("avg successes")
    plt.ylabel("flammability q")
    plt.show()#graphs success vs flammability of CBS(my own strategy)


def plotProbDFS(dim):#runs trials of dfs and plots the points
    for p in range(100):
        trueCount=0
        for d in range(50):
            if(dfs(generateGrid(dim, p), Node(0, 0, 0), Node(dim-1, dim-1, 0), dim)):
                trueCount+=1
        if(trueCount!=0):
            plt.plot(float(trueCount)/50.0, p, "ob")
    plt.title("Obstacle density vs Chance to be able to make it to goal")
    plt.xlabel("Probability that S can be reached from G")
    plt.ylabel("Obstacle density p")
    plt.show()

def plotProbBFSandA(dim, start, goal):#runs trials of bfs and Astar and plots the points
    for p in range(100):
        sumaStar = 0
        sumBFS = 0
        avg = 0
        grid = generateGrid(dim, p)
        for d in range(10):

            booly, sumNodesVisitedaStar = aStar(grid, start, goal, dim)
            sumaStar = sumNodesVisitedaStar + sumaStar
            booly, sumNodesVisitedbfs = bfs(grid, start, goal, dim)
            sumBFS = sumNodesVisitedbfs + sumBFS

            avg = (sumBFS-sumaStar)/10.0

        plt.plot(avg, p, "ob")
    plt.title("Obstacle density vs Chance to be able to make it to goal")
    plt.xlabel("Number of nodes explored by BFS - number of nodes explored by A*")
    plt.ylabel("Obstacle density p")
    plt.show()





"""
madeIt, nodesVisited = aStar(generateGrid(20, 50), Node(0, 0, 0), Node(20-1, 20-1, 0), 20)
print("made it: "+str(madeIt)+" Nodes visited: "+str(nodesVisited))

madeIt, nodesVisited = aStar(generateGrid(20, 30), Node(0, 0, 0), Node(20-1, 20-1, 0), 20)
print("\n")

print("made it: "+str(madeIt)+" Nodes visited: "+str(nodesVisited))
"""
#plotProbDFS(20)


"""
print(grid)
grid = advanceFire(grid, .5)

grid = advanceFire(grid, .5)

grid = advanceFire(grid, .5)

grid = advanceFire(grid, .5)
grid = advanceFire(grid, .5)

grid = advanceFire(grid, .5)
grid = advanceFire(grid, .5)

grid = advanceFire(grid, .5)


grid = generateGrid(10, 30)

grid, node = setOnFire(grid)
#print("node: "+str(node.row)+", "+str(node.col))
#plotStrategy2(10, Node(0, 0, 0), Node(9, 9, 0))
print(grid)
print(frankStrategy(grid, Node(0, 0, 0), Node(9, 9, 0), .3))
plotFrankStrategy(10, Node(0, 0, 0), Node(9, 9, 0))
"""

#print(bfs(generateGrid(100, 30), Node(0, 0, 0), Node(99, 99, 0), 100))

grid=generateGrid(20,30)
setOnFire(grid)
strategy1(grid, Node(0, 0, 0), Node(19, 19, 0), .2)

#print(strategy1(grid, Node(0, 0, 0), Node(9, 9, 0), .3))








#plotProbBFSandA(20, Node(0,0,0), Node(19,19,0))
