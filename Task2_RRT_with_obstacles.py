import numpy as np
import matplotlib.pyplot as plt
from numpy import random

class Node:
    def __init__(self, position, parent, children):
        self.position = position
        self.parent = parent
        self.children = children

class RRT:
    def __init__(self, init_position, target, delta_d = 1, planning_domain = [100,100], center = [], radius = []):
        self.init_position = init_position          #initial configuration
        self.target = target
        self.delta_d = delta_d                      #incremental distance
        self.planning_domain = planning_domain      #the two-dimensional planning domain
        self.center = center
        self.radius = radius

    def set_environment(self, ax):
        #fig, ax = plt.subplots(figsize = self.planning_domain)
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)   
        #create 35 circle obstacles with random radius and random position   
        for i in range(35):
            self.center.append([random.randint(0, 100), random.randint(0, 100)])
            self.radius.append(random.randint(0, 10))
            draw_circle = plt.Circle(self.center[i], self.radius[i], color = "black")
            ax.set_aspect(1)
            ax.add_artist(draw_circle)
        # inital position and target needs to be outside of obstacles
        for i in range(35):
            dis_init = np.sqrt((self.init_position[0]-self.center[i][0])**2 + (self.init_position[1]-self.center[i][1])**2)
            dis_tar = np.sqrt((self.target[0]-self.center[i][0])**2 + (self.target[1]-self.center[i][1])**2)
            if dis_init < self.radius[i] or dis_tar < self.radius[i]:
                exit()

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
            print(dis)
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

    #check the distance between a point the a line and check if the line between near_pos and new_pos cross circles(obstacles)
    def point_line(self, near_pos, new_point, center, radius):
        for i in range(len(center)):
            vector_near = (near_pos[0] - center[i][0], near_pos[1] - center[i][1])
            V_near = np.array(vector_near)
            dis_near = np.sqrt(np.dot(V_near,V_near))
            vector_new = (center[i][0] - new_point[0], center[i][1] - new_point[1])
            V_new = np.array(vector_new)
            dis_new = np.sqrt(np.dot(V_new,V_new))

            if dis_near <= self.radius[i] or dis_new <= self.radius[i]:
                return False
        return True

    #Check if the node can connect to the target position without touch any obstacles.
    def direct_line(self, new_point, target, center, radius):
        for i in range(len(self.center)):    
            p1 = np.array(new_point)
            p2 = np.array(target)
            p3 = np.array(center[i])
            u = np.dot(p3-p1, p2-p1)/np.dot(p2-p1,p2-p1)
            x = p1[0] + u*(p2[0]-p1[0])
            y = p1[1] + u*(p2[1]-p1[1])
            p4 = np.array((x,y))
            dis = np.sqrt(np.dot(p3-p4,p3-p4))
            if dis < radius[i] and 0 <= u <= 1:
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
    def generate_RRT(self, K, ax):
        q_init = Node(self.init_position, 0, None) #the root
        node = [q_init] #create a list to include all element inside the tree

        plt.plot(self.init_position[0], self.init_position[1], marker = "o", markersize = 4, color = 'red')
        plt.plot(self.target[0], self.target[1], marker = "o", markersize = 4, color = 'green')
        for k in range(K):
            rand_pos = self.q_rand()
            location = self.q_near(rand_pos, node) #return the location of the nearest point
            near_pos = node[location].position     
            new_point = self.q_new(near_pos, rand_pos) #this is position of the new point  
            avoid = self.point_line(near_pos, new_point, self.center, self.radius) #check to avoid obstacles
            new_pos = Node(new_point, location, None)
            if avoid:
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
            plt.pause(0.001)

            # test if RRT reach the target position from the new point directly go to target and plot the line
            end_line = self.direct_line(new_point, self.target, self.center, self.radius)
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

        x_path = [i[0] for i in path]
        y_path = [i[1] for i in path]
        plt.plot(x_path,y_path, 'red', markersize = 2)
        plt.show()


def main():
    init_position = (random.randint(0, 100), random.randint(0, 100))
    target = (random.randint(0, 100), random.randint(0, 100))

    test = RRT(init_position, target)

    fig, ax = plt.subplots()
    test.set_environment(ax)
    test.generate_RRT(1000,ax)



if __name__=='__main__':
    main()

