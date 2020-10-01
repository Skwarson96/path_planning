# path_planning

The repository is an extension of the homework in the subject of Methods and Algorithms of Motion Planning. 
The aim of the task was to implement space searching algorithms

### 1. RRT vertices
Rapidly-exploring Random Trees algorithm

launch with:
- $ roslaunch path_planning rrt_vertices.launch

youtube link:
- ----------

### 2. RRT
Rapidly-exploring Random Trees algorithm

launch with:
-  $ roslaunch path_planning rrt.launch

youtube link:
- -----------

### 3. PRM
Probabilistic Road Map algorithm
You can choose between two types of Neighbor Finding. By radius and k nearest neighbors
```python
        # find closest points
        prm.find_closest_in_radius()
        # k nearest points
        prm.find_k_nearest()
```
launch with:
- $ roslaunch path_planning prm.launch

youtube link:
- -----------

### 4. RRT star
Rapidly-exploring Random Trees star algorithm

launch with:
- $ roslaunch path_planning rrt_star.launch

youtube link:
- -----------


