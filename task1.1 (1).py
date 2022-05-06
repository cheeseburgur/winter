import cv2
import numpy as np
import random
from collections import deque
import time

mat = np.full((10, 10), 255, dtype=np.uint8)
n, l = mat.shape

for i in range(int(0.2 * n * l)):
    x = random.randint(0, n - 1)
    y = random.randint(0, l - 1)
    mat[x, y] = 0

mat[2, 2] = 127
mat[8, 8] = 127

mat1 = np.full((100, 100), 0, dtype=np.uint8)
cv2.resize(mat, (100, 100), mat1, None, None, interpolation=cv2.INTER_AREA)
#starting and ending point
start = (20, 20)
end = (80, 80)

for i in range(20, 30):
    for j in range(20, 30):
        mat1[i][j] = 255

for i in range(80, 90):
    for j in range(80, 90):
        mat1[i][j] = 255

mat1[20, 20] = 127
mat1[80, 80] = 127

red=(0,0,255)
green=(0,255,0)
grey=(127,127,127)
black=(0,0,0)
orange=(0,128,255)
pink=(255,0,255)

#mat1=np.full((50,50),0,dtype=np.uint8)
#cv2.resize(mat,(50,50),mat1,None,None,interpolation=cv2.INTER_AREA)
mat1=cv2.cvtColor(mat1,cv2.COLOR_GRAY2BGR)
mat1_copy=mat1.copy()
n,l,m=mat1.shape

def calcDist(point,current):
    return (point[0] - current[0])**2 + (point[1] - current[1])**2
    
def iswell(x,y):
    return (x>=0 and x<mat1.shape[0] and y >=0 and y<mat1.shape[1])

def isObstacle(p):
    x,y=p
    if mat1[x,y][0]==black[0] and mat1[x,y][1]==black[1] and mat1[x,y][2]==black[2]:
        return True
    return False
         
def dijkstra(start,end):
    h,w,t = mat1.shape
    dist = np.full((h,w), fill_value= np.inf)
    dist[start] = 0
    parent = np.zeros((h,w,2))
    visited = np.zeros((h,w))
    current = start
    visited[start] = 1
    while current != end:
        print(current)
        visited[current] = 1
        for i in range(-1,2):
            for j in range(-1,2):
                point = (current[0]+i,current[1]+j)
                if iswell(point[0],point[1]) and visited[point] != 1 and isObstacle(point)==False:
                    if calcDist(point,current) + dist[current]  < dist[point]:
                        dist[point] = calcDist(point,current) + dist[current] 
                        parent[point[0],point[1]] = [current[0],current[1]]
        min = np.inf
        for i in range(h):
            for j in range(w):
                if min > dist[i,j] and visited[i,j] != 1:
                    min = dist[i,j]
                    current = (i,j)
        showPath(current,start,parent)   
            
def showPath(current,start,parent):
    new=mat1.copy()
    while current != start:
        var = int(parent[current][0]) , int(parent[current][1])
        
        new[int(var[0]),int(var[1])] = green
        current = (var[0],var[1])

    cv2.namedWindow('windowdijkstra',cv2.WINDOW_NORMAL)
    cv2.imshow('windowdijkstra',new)
    cv2.waitKey(1)

print("Start:",start)
print("End:",end)

dbegin_time=time.time()
dijkstra(start,end)
dend_time=time.time()


cv2.namedWindow("Finaldijkstra",cv2.WINDOW_NORMAL)
cv2.imshow("Finaldijkstra",mat1)
cv2.waitKey(0)
print("Dijkstra time:",dend_time-dbegin_time)

class Node():
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        self.g = np.inf
        self.h = np.inf
        self.f = np.inf

def get_min_dist_node(open_list):
    min_dist = np.inf
    min_node = None
    for node in open_list:
        if open_list[node].f < min_dist:
            min_dist = open_list[node].f
            min_node = open_list[node]
    return min_node

def show_path(node):
    print('show path')
    current_node = node
    path = []
    while current_node is not None:
        path.append(current_node.position)
        current_node = current_node.parent
    path.reverse()
    for i in range(len(path)-1):
        cv2.line(mat1_copy, path[i], path[i+1], green, 2)
    cv2.namedWindow('final astar', cv2.WINDOW_NORMAL)
    cv2.imshow("final astar", mat1_copy)
    cv2.waitKey(0)
    print("Final image shown")
    cv2.imwrite("final_path.png", mat1_copy)
    #if cv2.waitKey(1) == 'q':
    #    cv2.destroyAllWindows()
    #    return

def astar_algorithm(start, end):
    print('astar called')
    open_list = {}
    closed_list = []
    start_node =  Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    open_list[start] = start_node
    while len(open_list)>0:
        print("dict size = ", len(open_list))
        current_node = get_min_dist_node(open_list)
        mat1[current_node.position[1]][current_node.position[0]] = orange
        open_list.pop(current_node.position)

        if current_node.position == end:
            print("Goal Reached")
            show_path(current_node)
            return

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0] > (n - 1) or node_position[0] < 0 or node_position[1] > (l - 1) or node_position[1] < 0:
                continue
            if node_position in closed_list:
                continue
            if isObstacle(node_position):
                continue
            
            mat1[node_position[1]][node_position[0]] = pink
            new_node = Node(current_node, node_position)

            new_node.g = current_node.g + calcDist(current_node.position, new_node.position)
            new_node.h = calcDist(new_node.position, end)
            new_node.f = new_node.g + new_node.h

            if new_node.position in open_list:
                if new_node.g < open_list[new_node.position].g:
                    open_list[new_node.position] = new_node
            else:
                open_list[new_node.position] = new_node
        
        if current_node.position not in closed_list:
            closed_list.append(current_node.position)

abegin_time = time.time()
astar_algorithm(start, end)
aend_time = time.time()

print("astar time = ", (aend_time-abegin_time))
print("Start=",start)
print("End=",end)
cv2.destroyAllWindows()    