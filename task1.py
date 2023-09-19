import numpy as np
import matplotlib.pyplot as plt
from numpy import random

class Node:
    def __init__(self, position, parent = None, clildren = None):
        self.position = position
        self.parent = parent
        self.clildren = clildren

class RRT:
    def __init__(self, init_position = [50,50], delta_d = 1, planning_domain = [100,100]):
        self.init_position = init_position          #initial configuration
        self.delta_d = delta_d                      #incremental distance
        self.planning_domain = planning_domain      #the two-dimensional planning domain

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
        return node[point].position #the return is a list

    #generate a new configuration
    def q_new(self, near_vertex, rand_posi):
        dis = np.sqrt((rand_posi[0]-near_vertex[0])**2+(rand_posi[1]-near_vertex[1])**2) #distance between random position and the current point
        vector = ((rand_posi[0]-near_vertex[0]), rand_posi[1]-near_vertex[1]) #vector points to the random position
        
        V = np.array(vector) 
        N = np.array(near_vertex) 

        new_pos = N + self.delta_d/dis*V #display the new position, list
        
        return new_pos

    #RRT algorithm
    def generate_RRT(self, K):
        G = Node(self.init_position)
        node = [G] #create a list to include all element inside the tree
        fig, ax = plt.subplots(figsize = self.planning_domain)
        plt.plot(self.init_position[0], self.init_position[1], marker = "o", markersize = 3, color = 'blue')
        for k in range(K):
            rand_pos = self.q_rand()
            near_pos = self.q_near(rand_pos, node)          
            new_pos = Node(self.q_new(near_pos, rand_pos),G) #this is a Node
            node.append(new_pos)
            new_x = (near_pos[0],new_pos.position[0])
            new_y = (near_pos[1],new_pos.position[1])
            ax.plot(new_x, new_y, color = 'blue') #plot the line
            plt.plot(new_pos.position[0], new_pos.position[1], marker = "o", markersize = 3, color = 'blue') #plot the point
            ax.set_xlim(0, 100)
            ax.set_ylim(0, 100)
            plt.pause(0.001)
            plt.title(str(K+1)+" Iterations")
            plt.xlabel("X Range")
            plt.ylabel("Y Range")
        plt.show()

def main():
    test = RRT()
    test.generate_RRT(400)


if __name__=='__main__':
    main()

