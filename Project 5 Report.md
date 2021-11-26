# Task Planning Report

> Rui Xiao xrui0310@gmail.com

### Problem Statement and Motivation

For this project we are doing 5.6 Task Planning, that is to design an SAT planner to solve the "Sokoban on Ice" problem.

By solving this problem we will understand how to solve robotics problems using task planning, to model the problem into a task planning problem, and finding the optimal methods to solves these problems. Since task planning problems play a important role in industry world, and efficient task planning methods are the key to saving resources as well as increasing efficiency.

Moreover, Planning as Satisfiability is a classic but still challenging problem that continue to inspire the field of computer science. We are motivated to explore the methods and reasons behind this broadly applied field of study.

### Explanation of Approach

#### Why SAT

As we all know, there are various methods in planning and solving for task planning problems. Our motivations to solve this problem with a SAT planner include:

1. This problem can be easily modeled into a SAT problem.
2. Planning as satisfiability makes it easy to state random facts about any of the states of the problem, and it is easy to set random constraints on the problem.
3. There exists a convenient Z3 theorem solver that makes implementing SAT planning easier and prettier.

#### How to Solve with SAT

The process of having the SAT-planner to solve the task planning problem could be considered as the following steps:

1. Fetch and Process the Input Scene: We implemented this part in `InputBundle` helper class.
2. Initializing Propositions: We used a separate `Variable` class to store the generated propositions.
3. Generate Formula: Including conjunctions for the Initial State, Goal State, Operation Encodings, Frame Axioms and Complete exclusion axiom.
4. Find an Assignment using SAT-solver: This could be automatically done by calling `solver.check()`. Then the `solver.model()` indicates the solution path if one is found.

Planning for the problem without movable obstacles is easy. There are no vary situations that need to be considered as when to move the movable obstacles and at which direction to move the movable obstacles. It is not difficult to generate a formula constructed with a set of actions, goal, and horizon length such that the formula is satisfiable if and only if there is a plan with the given horizon. However, designing a planner for the version with movable obstacles is much more challenging.  

To consider the movable obstacles, the encoder would go through all the possible combinations of the positions of the movable obstacles. With each case, the operation encodings and frame axioms are properly set. It will consider the change of each cells after the movement of the robot. By this means, the propositions is defined to satisfy the rule of the game. Then the SAT solver just need to find the assignment for the formula which is supposed to be the solution path.

#### Optimization

The main optimization method of using SAT-palnner to solve a problem is to reduce the number of propositions when encoding, hence reducing the size of the SAT formula. With smaller SAT formula, the SAT solver would find the assginment or find out that no satisfying assignment exists more quickly.

We have employed three main ways to reduce the number of propositions.

##### Avoid Propositions in Obstacle Cells

When generating the conjunctions of operations and frame axioms, we tried to avoid generating the useless propositions within the cells where an obstacle exists. For the cases where there are lots of obstacles, this could considerably reduce the number of propositions.

##### Global Move Operation

The naive way of encoding the move operation of the robot is to have separate move propositions for each of the cells in the workspace with each of the four directions. But combining move operation with robot position is unnecessay. We have used only four propositions for each of the for directions of the robot movement. When we will need to encode the movement, we just encode it with the position of the robot together. By this means, the number of proposition for the robot movement could be reduced from $4\times MN$ to only $4$, which is completely independent from the position of the robot.

##### Optimized Robot Position Presentation

The naive way of encoding the position of the robot is to have one proposition for each of the cell and each proposition represents one possible position of the robot. Then there would be $M\times N$ propositions needed. We used $M$ propositions to represent the $x$ axis position of the robot and $N$ propositions to represent the $y$ axis position of the robot. Then there would be only $M+N$ propositions needed. This is a huge optimiztion when the scale of the workspace is large.

#### Breadth First Search

Both python and c++ version of the BFS algorithm have been implemented.
For the unmovable python algorithm, the average runtime is around $0.036$s (50 rounds). For the movable python algorithm, the average runtime is around $0.057$s (50 rounds, 2 scenes combined). While the BFS algorithm generally runs faster than our SAT algorithm, and that the unmovable python algorithm provides 100% correct solution, it should be noted that the movable python algorithm works fine with scene 1 but fails to find the solution for scene 2. That is because an optimal algorithm will decide to not push the movable obstacle on $[1,4]$ while the BFS algorithm doesn't hesitate to do so. A better algorithm is needed to fix this problem.
As shown in the graph, the python algorithm takes input that is modified from the given scene, and outputs the BFS movement each step, the solution, and the tree. In the screenshots, 'v' stands for visited nodes, 'o' for obstacles, 'm' for movables, and 'g' for goal.

<center><img src="Project 5 Report.assets/15491637683401_.pic_hd.jpg" alt="15491637683401_.pic_hd" style="zoom:40%;" /></center>
<center><img src="Project 5 Report.assets/15481637683401_.pic_hd.jpg" alt="15481637683401_.pic_hd" style="zoom:33%;" /></center>

<center style="font-size:14px">Figure 1. BFS in Python</center> 

### Description of Experiments

#### Robot

In the experiment we designed, the robot is tasked with reaching a target cell in the grid world. It can move in any of the four cardinal directions (up, down, left, right). When it starts moving, it can not stop until it hits a static obstacle or the workspace boundary.

#### Environment

The environment is a fixed size rectangular bound that contains fixed size triangles. Each triangle represents either the robot, the goal, an empty space, an static obstacle, or a movable ice. One way to solidate the problem is to model the boundary into static obstacles as well.

#### Runtime Platform

The experiment is conducted on a macbook based on *macOS 12.0.1*. The following table shows the hardware and software specification of the runtime environment.

|                  | Information  |
| ---------------- | ------------ |
| CPU              | i7 2.6 GHz   |
| RAM              | 16GB 2667MHz |
| Compiler         | Clang 13.0.0 |
| Operating System | macOS 12.0.1 |
| Z3 Solver        | 4.8.13 64bit |
| OpenCV           | 3.4          |

<center style="font-size:14px">Table 1. Paltform Specification</center> 

### Analysis

#### Example Solution

##### Ice Cave

<img src="Project 5 Report.assets/截屏2021-11-23 04.33.29.png" alt="截屏2021-11-23 04.33.29" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.31.png" alt="截屏2021-11-23 04.33.31" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.34.png" alt="截屏2021-11-23 04.33.34" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.36.png" alt="截屏2021-11-23 04.33.36" style="zoom:16%;" />
<img src="Project 5 Report.assets/截屏2021-11-23 04.33.38.png" alt="截屏2021-11-23 04.33.38" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.40.png" alt="截屏2021-11-23 04.33.40" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.43.png" alt="截屏2021-11-23 04.33.43" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.45.png" alt="截屏2021-11-23 04.33.45" style="zoom:16%;" />
<img src="Project 5 Report.assets/截屏2021-11-23 04.33.47.png" alt="截屏2021-11-23 04.33.47" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.50.png" alt="截屏2021-11-23 04.33.50" style="zoom:16%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.33.52.png" alt="截屏2021-11-23 04.33.52" style="zoom:16%;" />

<center style="font-size:14px">Figure 2. Ice Cave Solution</center> 

##### Scene 1

<img src="Project 5 Report.assets/截屏2021-11-23 04.28.22.png" alt="截屏2021-11-23 04.28.22" style="zoom: 17%" /><img src="Project 5 Report.assets/截屏2021-11-23 04.28.26.png" alt="截屏2021-11-23 04.28.26" style="zoom:17%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.28.28.png" alt="截屏2021-11-23 04.28.28" style="zoom:17%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.28.31.png" alt="截屏2021-11-23 04.28.31" style="zoom:17%;" />
<img src="Project 5 Report.assets/截屏2021-11-23 04.28.38.png" alt="截屏2021-11-23 04.28.38" style="zoom:17%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.28.41.png" alt="截屏2021-11-23 04.28.41" style="zoom:17%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.28.44.png" alt="截屏2021-11-23 04.28.44" style="zoom:17%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.28.47.png" alt="截屏2021-11-23 04.28.47" style="zoom:17%;" />

<center style="font-size:14px">Figure 3. Scene 1 Solution</center> 

##### Scene 2

<img src="Project 5 Report.assets/截屏2021-11-23 04.37.22.png" alt="截屏2021-11-23 04.37.22" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.24.png" alt="截屏2021-11-23 04.37.24" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.26.png" alt="截屏2021-11-23 04.37.26" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.31.png" alt="截屏2021-11-23 04.37.31" style="zoom:18%;" />

<img src="Project 5 Report.assets/截屏2021-11-23 04.37.34.png" alt="截屏2021-11-23 04.37.34" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.36.png" alt="截屏2021-11-23 04.37.36" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.39.png" alt="截屏2021-11-23 04.37.39" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.42.png" alt="截屏2021-11-23 04.37.42" style="zoom:18%;" />

<img src="Project 5 Report.assets/截屏2021-11-23 04.37.44.png" alt="截屏2021-11-23 04.37.44" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.46.png" alt="截屏2021-11-23 04.37.46" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.48.png" alt="截屏2021-11-23 04.37.48" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.51.png" alt="截屏2021-11-23 04.37.51" style="zoom:18%;" />

<img src="Project 5 Report.assets/截屏2021-11-23 04.37.54.png" alt="截屏2021-11-23 04.37.54" style="zoom:18%;" /><img src="Project 5 Report.assets/截屏2021-11-23 04.37.56.png" alt="截屏2021-11-23 04.37.56" style="zoom:18%;" />

<center style="font-size:14px">Figure 4. Scene 2 Solution</center> 

#### Benchmark

##### Primary Cases

###### Time

We have measured the performance of the SAT-plan solver with the given three cases: *Ice Path*, *Scene 1*, and *Scene 2*. The following figure shows the time consumption of 50 runs for each scene.

<img src="Project 5 Report.assets/boxplot-light-velocity-2-7652308.png" alt="boxplot-light-velocity-2" style="zoom:28%;" />

<center style="font-size:14px">Figure 5. Time Consumption</center> 

The *Ice Path* scene is the easiest one for the solver and it costs the solver extremely short time to find a solution. It also works pretty good in *Scene 1*. The time consumption is slightly higher than previous *Ice Path* scene. This may be caused by the enlarged workspace sacle and a movable obstacle. The solver still works good enough since it could find a solution in no more than $0.1$ second most of the time. It works really slowly in *Scene 2* compared to the performance with the other two scenes. This is mainly caused by the increase of the number of movable obstacles and the depth of the potential solutions.

###### Memory

To measure the memory consumption of the solver, we fetched the statistics information of the z3 solver and analyzed two metrics from it: memory and boolean variable number. The result is shown below:

|          | Memory | Boolean Variable Number |
| -------- | ------ | ----------------------- |
| Ice Path | 22.02  | 1830                    |
| Scene 1  | 24.42  | 2219                    |
| Scene 2  | 56.56  | 11912                   |

<center style="font-size:14px">Table 2. Memory Consumption</center> 

The result of memory consumption is similar to the time consumption we have discussed above. The solver cost the least memory and generated fewest boolean variables when solving the easiest *Ice Path* scene. *Scene 1* cost slightly more. The last scene *Scene 2* is much harder to solve. It took much more memory to solve it.

###### Success Ratio

The success ratio of the solver to find a solution completely depends on the maximum step configuration. The following three figures shows the results when the maximum step is limited to $1$, $10$, and $20$.

<center><img src="Project 5 Report.assets/boxplot-light-velocity-15.png" alt="boxplot-light-velocity-15" style="zoom:21%;" /><img src="Project 5 Report.assets/boxplot-light-velocity-14.png" alt="boxplot-light-velocity-14" style="zoom:21%;" /><img src="Project 5 Report.assets/boxplot-light-velocity-13.png" alt="boxplot-light-velocity-13" style="zoom:21%;" /></center>

<center style="font-size:14px">Figure 6. Success Ratio</center> 

The minimum step numbers of the solutions in the three scenes are $10$, $7$, and $13$ respectively. The result of the sucess ratio indicates that as long as the maximum step is set to be higher than the step number of the potential solution, the solver would find it. This is because the SAT-planner is guranteed to find the assignment of the propositions corresponding to the solution as long as there exists one.

###### Stability

We have defined the stability of our SAT-planner as the variation of time it costs between separate but identical attempts. The following figure shows the stability of 50 runs in each case.

<img src="Project 5 Report.assets/boxplot-light-velocity-17.png" alt="boxplot-light-velocity-17" style="zoom:28%;" />

<center style="font-size:14px">Figure 7. Stability</center> 

We have cut the range of the runtime samples into $11$ pieces and put each runtime into the buckets. $y$ axis shows how many samples belongs to this bucket. The runtime in case *Ice Cave* and case *Scene 1* are distributed similarly. There is a peak near the front middle part of the distribution. But there is no such kind of peak showing the aggregated distribution in case *Scene 2*. This means the solver is less stable solving *Scene 2*. It is harder to estimate how much time it would take to find the solution.

##### Workspace Scale

We have also designed two variant scenes based on *Scene 1* with different workspace scale. As shown in the following figures, *Scene 1 enlarged scale* is sized $12\times11$ and *Scene 1 very large scale* is sized $16\times15$. But the solution path depth and the numbers of movable osbtacles have been kept the same. This figure shows how the runtime changed due to the change of workspace scale.

<img src="Project 5 Report.assets/boxplot-light-velocity-23.png" alt="boxplot-light-velocity-23" style="zoom:28%;" />

<center style="font-size:14px">Figure 8. Time Consumption with Different Workspace Scales</center> 

The result above corresponds to the comparison we have made between *Ice Cave*, *Scene 1*, and *Scene 2*. The sacle of the workspace would exponentially increase the runtime of the search even with the same solution depth and movable obstacle number. This result corresponds to the intuition that the number of cells in the workspace increases exponentially when the workspace scale increases.

##### Movable Obstacle Number

To measure the influence of the number of movable obstacles to the performance of the solver, we have designed two variant scenes based on *Scene 1*. One of them has $3$ movable obstacles and the other has $5$ obstacles. Both of them has completely same solution with the original *Scene 1*. The following figure shows the results.

<img src="Project 5 Report.assets/boxplot-light-velocity-24.png" alt="boxplot-light-velocity-24" style="zoom:28%;" />

<center style="font-size:14px">Figure 9. Time Consumption with More Movable Obstacles</center> 

The result of this comparison is similar to the comparison with change on workspace sacle. The increase of the number of movable obstacles would cause an increase on the runtime. But the increase seems to be linear rather than exponential. More movable obstacles would cause more propositions when generating the SAT formula. This result corresponds to the encoding process we have implemented.

##### Compared with BFS

We have tried BFS method to search the solution in the given cases. The following firgue shows the time consumption of BFS and SAT-planner in *Scene 2*.

<img src="Project 5 Report.assets/boxplot-light-velocity-21.png" alt="boxplot-light-velocity-21" style="zoom:28%;" />

<center style="font-size:14px">Figure 10. Time Consumption of Different Methods</center> 

As shown in the figure, BFS works much faster than the SAT-plan solver. With hash table acceleration, BFS search is even faster. This may be caused by the complicated process of generating the operation encodings and frame axioms for the SAT formula.

<img src="Project 5 Report.assets/boxplot-light-velocity-20.png" alt="boxplot-light-velocity-20" style="zoom:28%;" />

<center style="font-size:14px">Figure 11. Stability of Different Methods</center> 

The stability of SAT-plan is worse than the normal BFS search. Not mention the accelerated BFS search that has a main peak with more than $80\%$ of samples aggregated tightly. We think the main reason why SAT-plan works with poor stability is that the SAT solver relies highly on the z3 module, which is complicated and with no proof of reliability.

#### Optimality

The optimality here we discuss is mainly about how the solution path is optimal and why it is guranteed to be optimal.

The solution path found by the SAT-planner has the minimum step number. As we have seen in the success ratio part above, the SAT-plan solver would always be able to find out the solution path within the given step limit as long as there exists one. We have implemented the solver to iteratively increase the limit from $1$ to the maximum attempt number and try to find an assignment of the propositions with each step limit. If there exists a solution, it would find it once the step limit is increased to be the same as the potential solution. This could gurantee that the solution with the minimum step number would be found by the solver. Then the optimality is guranteed.

#### Completeness

The SAT-planner has discretization completeness.

- If there is a solution with finite depth, it would find it as long as the maximum step limit has been increased to the depth of the potential solution after many iterations.
- If there is no solution, it would not be able to find out. It would iterate with step limit increasing infinitely or stop with the maximum iterate configuration.

### Difficulties of Exercises

- Naive Encoding without Movable Obstacles: 7, 12h;

- Optimized Encoding with Movable Obstacles: 10, 11h;
- BFS Search: 4, 4h;
- Benchmark: 3, 3h.

We think the hardest part of this assignment is to understand how to encode the problem into a solvable SAT problem for the SAT solver Z3. It really took us a lot of time understanding the Operation Encodings and Frame Axioms. The movable obstacles are hard to be encoded.
