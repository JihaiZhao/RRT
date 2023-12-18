from imageio.v2 import imread
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from numpy import random
import cv2 as cv

class Node:
    def __init__(self, position, parent, children):
        self.position = position
        self.parent = parent
        self.children = children

class RRT:
    def __init__(self, init_position = [40,40], target = [60,60], delta_d = 1, planning_domain = [100,100], bg = np.flipud(imread('N_map.png'))):
        self.init_position = init_position          #initial configuration
        self.target = target
        self.delta_d = delta_d                      #incremental distance
        self.planning_domain = planning_domain      #the two-dimensional planning domain
        self.bg = bg

    def set_environment(self, ax):
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        plt.imshow(self.bg, cmap = 'gray', origin='lower')

    #generate a random position in the domain
    def q_rand(self):
        position = [random.rand()*self.planning_domain[0], random.rand()*self.planning_domain[1]]
        return position

    #finds the vertex in G that is closest to the given position
    def q_near(self,rand_posi, node):
        point = 0
        dis_near = 100
        for i in range(len(node)):
            dis = np.sqrt((rand_posi[0]-node[i].position[0])**2+(rand_posi[1]-node[i].position[1])**2)
            if dis < dis_near:
                dis_near = dis
                point = i
        return  point 

    #generate a new configuration
    def q_new(self, near_vertex, rand_posi):
        dis = np.sqrt((rand_posi[0]-near_vertex[0])**2+(rand_posi[1]-near_vertex[1])**2)
        vector = ((rand_posi[0]-near_vertex[0]), rand_posi[1]-near_vertex[1])
        V = np.array(vector) #vector
        N = np.array(near_vertex) 
        new_pos = tuple(N + self.delta_d/dis*V) #display the new position, list
        return new_pos

    def check_collision(self, new_point):
        x1 = int(np.ceil(new_point[0]))
        x2 = int(np.floor(new_point[0]))
        y1 = int(np.ceil(new_point[1]))
        y2 = int(np.floor(new_point[1]))
        
        if ((tuple(self.bg[y1][x1]) == (0,0,0) or tuple(self.bg[y1][x2]) == (0,0,0)) or (tuple(self.bg[y2][x1]) == (0,0,0) or tuple(self.bg[y2][x2]) == (0,0,0))):
            return False
        else:
            return True

    #Check if the node can connect to the target position without touch any obstacles.
    def direct_line(self, new_point, target):
        #find the closest grid
        x1 = int(np.floor(new_point[0]))
        y1 = int(np.floor(new_point[1]))
        x2 = int(np.floor(target[0]))
        y2 = int(np.floor(target[1]))  

        while x1 != x2 or y1 != y2:
            if abs(x1-x2) < abs(y1-y2):
                y_increment = np.sign(y2-y1)
                if x1-x2 == 0:
                    y1 += y_increment
                    if tuple(self.bg[y1][x1]) == (0,0,0):
                        return False
                else:
                    y1 += y_increment
                    x_increment = np.sign(x2-x1)
                    x1 += x_increment
                    if tuple(self.bg[y1][x1]) == (0,0,0):
                        return False
            else:
                x_increment = np.sign(x2-x1)
                #print(x_increment)
                #print([x1,y1])    
                #print(self.bg[y1][x1])
                if y1-y2 == 0:
                    x1+=x_increment
                    if tuple(self.bg[y1][x1]) == (0,0,0):
                        return False
                else:
                    x1 += x_increment
                    y_increment = np.sign(y2-y1)
                    #print(y_increment)
                    y1 += y_increment
                    if tuple(self.bg[y1][x1]) == (0,0,0):
                        return False  
        return True
    
    #when RRT algorith find the path, traverse the tree to show the path
    def traverse_tree(self,node):
        while node.parent != None:
            x = (node.position[0],node.parent.position[0])
            y = (node.position[1],node.parent.position[1])
            plt.plot(x, y, color = 'red')
            node = node.parent

    #RRT algorithm
    def generate_RRT(self, K):
        fig, ax = plt.subplots()
        self.set_environment(ax)
        q_init = Node(self.init_position, 0, None) #the root
        node = [q_init] #create a list to include all element inside the tree
        a = 0
        plt.plot(self.init_position[0], self.init_position[1], marker = "o", markersize = 4, color = 'red')
        plt.plot(self.target[0], self.target[1], marker = "o", markersize = 4, color = 'green')
        for k in range(K):
            rand_pos = self.q_rand()
            location = self.q_near(rand_pos, node) #return the location of the nearest point
            near_pos = node[location].position     
            new_point = self.q_new(near_pos, rand_pos) #this is position of the new point  
            avoid = self.check_collision(new_point) #check to avoid obstacles     
            #print(avoid)       
            new_pos = Node(new_point, location, None)
            
            if avoid == True:
                a+=1
                x1 = int(np.ceil(new_point[0]))
                y1 = int(np.floor(new_point[1]))
                #print(x1,y1)
                #print(self.bg[x1][y1])
                node.append(new_pos)
                new_x = (near_pos[0],new_pos.position[0])
                new_y = (near_pos[1],new_pos.position[1])
                plt.plot(new_x, new_y, color = 'blue') #plot the line
                plt.plot(new_pos.position[0], new_pos.position[1], marker = "o", markersize = 1, color = 'blue') #plot the point
            #plot the path  
            ax.set_xlim(0, 100)
            ax.set_ylim(0, 100)
            plt.title(str(k+1)+" Iterations")
            plt.xlabel("X Range")
            plt.pause(0.01)

            # test if RRT reach the target position from the new point directly go to target and plot the line
            end_line = self.direct_line(new_point, self.target)
            #print(new_point)
            #print(end_line)
            if end_line:
                path_x = (new_point[0], self.target[0])
                path_y = (new_point[1], self.target[1])
                plt.plot(path_x, path_y)
                break

        # use red line to show the path from start position to the target position
        # create a tree from target to start position

        target_pos = Node(self.target, -1, 'end') 
        path = [target_pos.position]
        q_next = Node(node[-1].position, node[-1].parent, None) 
        while True:
            if q_next.position == q_init.position:
                break
            else:
                path.append(q_next.position)
                q_next = Node(node[q_next.parent].position, node[q_next.parent].parent, None)
        #print(a)
        x_path = [i[0] for i in path]
        y_path = [i[1] for i in path]
        plt.plot(x_path,y_path, 'red', markersize = 2)

        plt.show()

def main():
    #When accessing an image as an array, rows correspond to y values and columns to x values.
 
    test = RRT()
    test.generate_RRT(4000)




if __name__=='__main__':
    main()
