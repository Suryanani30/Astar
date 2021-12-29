import numpy as np
from scipy.spatial import distance

class Mazecode:

    def __init__(self, parentnode=None, position=None): #for initializing parent node and position
        self.parentnode = parentnode
        self.position = position
        self.gn = 0   #cost from start node to current node
        self.hn= 0    #hueristic value from current node to end node
        self.fn= 0    #sum of cost and hueristic value

    def __eq__(self, other):
        return self.position == other.position

def shortest_path(currentnode,rc,cc):
    path = []
    result = [[-1 for i in range(cc)] for j in range(cc)]
    current = currentnode # will be final node here
    count =0
    while current is not None:
        path.append(current.position)
        current = current.parentnode
        count = count +1
    path = path[::-1] # reverse path so to get from 1st node to end node since path is the reverse array
    return count,path


def Astar(mazedata, cost, start, end,rc,cc):    
    startnode = Mazecode(None, tuple(start))
    startnode.gn = startnode.hn = startnode.fn = 0 # since we are already in start node
    goalnode = Mazecode(None, tuple(end))
    goalnode.gn = goalnode.hn = goalnode.fn = 0
    openlist = []  
    closedlist = []     
    openlist.append(startnode)
    #rc, cc = np.shape(mazedata)

    options = [[-1, 0 ], # go up
              [ 0, -1], # go left
              [ 1, 0 ], # go down
              [ 0, 1 ]] # go right

    while len(openlist) > 0:
        currentnode = openlist[0]
        currentindex = 0
        for i, node in enumerate(openlist):
            if node.fn < currentnode.fn:
              currentnode = node
              currentindex = i

        openlist.pop(currentindex)
        closedlist.append(currentnode)
        # After appending the node to the closed list we perform the goal test
        if currentnode == goalnode:
            return shortest_path(currentnode,rc,cc)

        possiblemove = []

        for canmove in options: 
            index = (currentnode.position[0] + canmove[0], currentnode.position[1] + canmove[1])
            if (index[0] > (rc - 1) or index[0] < 0 or  index[1] > (cc -1) or index[1] < 0):
                continue
            # edge cases
            if mazedata[index[0]][index[1]] != 1:   # not obstacle
                continue
            newnode = Mazecode(currentnode,index)
            possiblemove.append(newnode)

        for child in possiblemove:
           
            if len([completed for completed in closedlist if completed == child]) > 0:
                continue
            child.gn = currentnode.gn + cost
            #Hueristic function
            #euclidean distance is used here
            a = (child.position[0],child.position[1])
            b = (goalnode.position[0],goalnode.position[1])
            child.hn = distance.euclidean(a,b)
            child.fn = child.gn + child.hn
            if len([node for node in openlist if child == node and child.gn > node.gn]) > 0:
                # if child is already in openlist and also cost is optimal
                continue
            #otherwise we update the openlist for all possible childs using this for loop
            openlist.append(child)


if __name__ == '__main__':
     print("This program calculates the shortest path if exists from the 1st point of the matrix (0,0) to the last of the matrix(rr-1,cc-1)") 
     rc = int(input("Enter the number of rows in the maze matrix:"))
     cc = int(input("Enter the number of columns in the maze matrix:"))
     mazematrix = []
     print("Enter the entries rowwise such that obstacle node is denoted by 0 and free node is denoted by 1:")
     for i in range(rc):          
        arr =[]
        for j in range(cc):      
            arr.append(int(input()))
        mazematrix.append(arr)
     print("Note: Start state should not be an obstacle state!!!1")
     a,b = input("Enter the index of the start point with indexes seperated by space:").split()
     startpoint = [int(a)-1,int(b)-1] 
     a,b = input("Enter the index of the goal point with indexes seperated by space:").split()
     endpoint = [int(a)-1,int(b)-1] 
     cost = int(input("Enter the cost for movement from 1 node to the other:"))

     path = Astar(mazematrix,cost, startpoint, endpoint,rc,cc)
     print(path)


    
