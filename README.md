# Rapidly-Exploring Random Tree
A Rapidly-Exploring Random Tree (RRT) is a fundamental path planning algorithm in robotics. Path planning is the task of moving a robot from one location to another, while avoiding obstacles and satisfying constraints.

An RRT consists of a set of vertices, which represent configurations in some domain D and edges, which connect two vertices. The algorithm randomly builds a tree in such a way that, as the number of vertices n increases to ∞, the vertices are uniformly distributed across the domain D⊂Rn.

## Task 1: Simple RRT
* Implement an RRT in a two-dimensional domain, D=[0,100]×[0,100]. Use an initial configuration of qinit=(50,50) and Δ=1.

* Plot the result for a few different values of K

![task1](https://github.com/JihaiZhao/RRT/assets/99274626/571919e4-bd08-4dcd-8ff6-f97863d304d9)

## Task 2: Planning a Path with Obstacles
Compare to task 1 there are three modifications to make:
* Created 35 circle obstacles with random radius and random position to the domain.

* Collision Checking

* Once find a path from a node in the tree to the goal state, I can traverse the tree backwards to the starting location to find the path

![task2](https://github.com/JihaiZhao/RRT/assets/99274626/c4347e3b-00d2-4c08-85c3-66b11c8d3e89)

## Task 3: RRT with Arbitrary Objects
Now let's consider arbitrary objects, represented by black pixels in a binary image. I will load a binary image into script, and randomly choose starting and goal locations, and then plan a path.

![task3](https://github.com/JihaiZhao/RRT/assets/99274626/f684e4dc-7652-4082-b7bc-e6e18d03948d)