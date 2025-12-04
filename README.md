[README.md](https://github.com/user-attachments/files/23940405/README.md)
**ROBO 201 Report (RRT and PRM Motion Planning)**

**1\. Introduction**   
                                                               
This project realizes two sampling-based motion planning techniques: Rapidly-Exploring Random Tree (RRT) and Probabilistic Roadmap (PRM) . Both methods employ collision checking , random sampling , and graph/tree seach to locate a path from a start configuration to a goal configuration on a 2D environment with obstacles.  
RRT develops a tree by sampling points and expanding toward them. PRM initially samples points, connects them, and then searches a graph.

**2\. Implementation Overview** 

2.1 Abstract interface

	All planners inherit from the PathPlanner abstract class.  
	It provides:  
is\_collision\_free()  
set\_obstacles()  
gest\_path()  
Abstract:plan(), get\_planning\_time(), get\_num\_nodes()

2.2 RRT Implementation 

Main ideas:

1. Random sampling   
2. Nearest node search   
3.  Steer toward sample by fixed step  
4.  Build a tree   
5.  Stop when goal is reached 

2.3 PRM Implementation

1. Sample random free point  
2. Connect each point to k-nearest neighbors  
3. Build roadmap graph  
4. Use Dikstra’s shortest path

**3\.** **Test setup**

	workspace: 10 x 10  
	Obstacles: random shapes ( Rectangles, Circles, Polygons)  
Ensure start/goal are not inside obstacles  
100 random scenarios per test

Parameters tested:

RRT:  
	max\_iterations \= 1000,  5000, 10000  
PRM:  
	num\_samples \= 1000, 5000, 10000  
k\_neighbors \= best value found (10) 

**4.1 RRT Results**

| Max Iteration | Success Rate | Avg Time (s) | Avg Path Length |
| :---- | :---- | :---- | :---- |
| 1000 | 46% | 0.105 | 18.13 |
| 5000 | 99% | 0.214 | 19.21 |
| 10000 | 100% | 0.217 | 18.89 |

   
**4.2 PRM Results**

| Num Samples | Success Rate | Avg Time (s) | Avg Path Length |
| :---- | :---- | :---- | :---- |
| 1000 | 97% | 3.18 | 15.52 |
| 5000 | 100% | 23.96 | 14.37 |
| 10000 | 100% | 66.49 | 14.26 |

**5\. Bar Plots**

![Image](https://github.com/user-attachments/assets/266ed214-3410-41ae-afc7-5b1c6d74d563)

![Image](https://github.com/user-attachments/assets/c7908e2b-6435-4e2c-9e33-a4f9f66432d4)
**6\. Analysis**

6.1 Computational Efficiency

	RRT is much faster than PRM in all tests.  
	PRM time grows a lot when the number of samples increases (the graph gets larger).

6.2 Path Quality  
	PRM produces shorter and smppther paths.  
	RRT paths are longer because the tree grows greedily.

6.3 Success Rate

	Small RRT (1000 iterations) has low success.  
	RRT becomes reliable at 5000+ iterations.  
	PRM has high success even at 1000 samples.

6.4 Obstacle Density

	PRM handles dense obstacles better because of global roadmap.  
	RRT sometimes gets stuck between obstacles.

6.5 When to Use Each

	RRT \= fast, good for real-time robotics  
	PRM \= better path quality, good for offline planning

**7\. Bonus: 3D RRT Extension**  
	  
7.1 Configuration3D  
	A new class was created with attributes (x,y,z) and a 3D distance function.

7.2 RRT in 3D  
	Same steps as 2D (sampling, nearest node, steering)  
	Extended to 3D space  
	Produces a collision-free 3D path  
	Visualized using Matplotlib 3D plotting ( Axes3D )

![Image](https://github.com/user-attachments/assets/0f97f131-c756-4e70-95c6-03eb6a66b96c)

**ROBO 202 REPORT ( Polymorphism and OOP )**

**1\. Introduction**

	This part explains how object-oriented programming concepts were used in the   
	Motion planning code.  
	I used an abstract class (pathPlanner) and two subclasses (RRTPlanner and   
	PRMPlanner).  
	Polymorphism allows one function to work with different planner types without   
	Changing the code.

**2\. UML Diagram**

Classes used in the project:  
	  
	Configuration  
	Configuration3D  
	PathPlanner  
	RRTPlanner  
	PRMPlanner

Inheritance:  
	RRTPlanner → PathPlanner  
	PRMPlanner → PathPlanner  
	Configuration3D → Configuration  
![Image](https://github.com/user-attachments/assets/94079cf7-1d91-4fb0-963d-40851c5dbc3d)


**3\. Polymorphism Explanation** 

The abstract class PathPlanner defines the required function:\\

* plan()  
* get\_planning\_time()  
* get\_num\_nodes()

These methods must exist in every planner, but each planner implements them differently.  
Abstract method make sure all planners follow the same interface.

Benefits of polymorphic test\_planner()  
Works with any planner that inherits from PathPlanner.  
It does not need to know if the planner is RRT, PRM, or 3D.  
This makes the code flexible and simple.

**4\. Code Snippets** 

1. Abstract Class

![Image](https://github.com/user-attachments/assets/320f934f-d335-4283-b8a5-bbc1d0f89574)

2. RRTplanner Overriding plan ()  
     
![Image](https://github.com/user-attachments/assets/6ca8b07e-e472-4ff9-b500-16d7d8a6dfd3)

3. Polymorphic test\_planner()  
     

![Image](https://github.com/user-attachments/assets/70987761-2d5a-4718-95c5-c2dfe461011c)

**5\. Visualizations** 

RRT 10000 iteration figure 

![][image8]  
PRM 10000 samples figure 

![][image9]

**6\. Conclusion**   
	  
	This project combines algorithms with software engineering concepts.  
	RRT and PRM were implemented using OOP principles, following clean class structure   
	and polymorphic design.  
	Performance analysis shows that RRT is faster, while PRM gives better paths.  
The 3D extension demonstrates that system can be easily scaled to higher dimensions.



