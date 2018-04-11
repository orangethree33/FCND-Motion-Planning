# FCND - 3D Motion Planning
![Quad Image](./misc/enroute.png)


In this project I will integrate the techniques that I have learned throughout the last several lessons to plan a path through an urban environment. 

### Step 1: Download the Simulator
This is a new simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: Clone this Repository
```sh
git clone https://github.com/udacity/FCND-Motion-Planning
```
### Step 4: Test setup
The first task in this project is to test the [solution code](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) for the Backyard Flyer project in this new simulator. Verify that your Backyard Flyer solution code works as expected and your drone can perform the square flight path in the new simulator. To do this, start the simulator and run the [`backyard_flyer_solution.py`](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) script.

```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer_solution.py
```
The quad took off, fly a square pattern and land, just as in the previous project.  I spend a lot of time to run it well, finally ,everything went well as expected . ready to start work on this project!

### Step 5: Inspect the relevant files
`motion_planning.py` and `planning_utils.py`.contains the utils fot the former,and `motion_planning `contains the class MotionPlanning
about this file`colliders.csv`, which contains the 2.5D map of the simulator environment. 

### Step 6: Explain what's going on in  `motion_planning.py` and `planning_utils.py`
*  `motion_planning.py` and `planning_utils.py`.
* Task in the project and explain the differences about the two scripts
`motion_planning.py` is basically a modified version of `backyard_flyer.py` that leverages some extra functions in `planning_utils.py`. but in `motion_planning.py`use the A* to find the path to the goal we want,use the following method to implement A*

1.plan_path
2.path,_=a_star(gird,heuristic,gird_start,gird_goal)
and the `backyard_flyer.py`use the`calculate_box` and `local_waypoints` to find the path.

*how the functions provided in  `planning_utils.py`work

1.`a_star`
  it is based on the based and the heurist,`heuristic` function is used in addition to the cost penalty. The `heuristic` function determines the  h()h()  value for each cell based on the goal cell and the method chosen to determine it. The heuristic value can be the Euclidean distance between these cells 
  * >c  is the current cost
  * >g  is the cost function
  * >h  is the heuristic function
  then the new cost is c+g+h
  also it is use PriorityQueue data structure ,to select the lowest cost partial plan .
  
2.`Aciton`
  An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
  and in my code there are 8 actions and 2 costs
3.`vaild_action`

return a list of vaild actions given a grid and current code,if the out drone meet a obstacles ,the action the drone act will be define as the invalid action,or else we will get the vaild actions,that is a great way to make us to choose how to avoid the obstacle and take  the all the action we need to  find the goal.

4.`create_grid`

the grid in fact is a matric ,there are 2 situations
*  if there are obstacles in the gird ,the value is 1
*  or else is zero 0
and this function returns a grid representation of a 2d configuration space

5.prune_path
use the `collinearity_check` to prune the path ,just like the teacher said in the class ,if three points are collinear ,then we remove the second point from the path,and we remove repeatly until there are no points can be removed.

 `def prune_path(path):
      pruned_path=[p for p in path]
      i=0;
      while i<len(pruned_path)-2:  
            p1=point(pruned_path[i])
            p2=point(pruned_path[i+1])
            p3=point(pruned_path[i+2])
            if collinearity_check(p1,p2,p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i+=1
         return pruned_path`
         
         
     while i<len(pruned_path))-2. 
     if we have 11 points,we don't need to check the the number of 11,12,13,
     because the the 12 points and the 13 points even is not exists,that is what I understand
          
  6.collinearity_check
   use the determination of the matrix to determine whether the three points are collinear or not.
   
  #Try running `motion_planning.py` to see what it does. 
   To do this, first start up the simulator, then at the command line:
 
```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python motion_planning.py
```
See the quad fly a jerky path of waypoints to the northeast for about 10 m then land. 

---
---
---


### Step 7: Write your planner

>The planning algorithm is going to look something like the following:

- Load the 2.5D map in the `colliders.csv` file describing the environment.
- Discretize the environment into a grid or graph representation.
- Define the start and goal locations. You can determine your home location from `self._latitude` and `self._longitude`. 
- Perform a search using A* or other search algorithm. 
- Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
- Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0]). 


***


### Step 8: Write it up!
* add the diagonal motion
   ```
      the diagonal action with a cost of sqrt(2)
         WEST=(0,-1,1)
         EAST=(0,1,-1)
         SORTH=(1,0,1)
         NORTH=(-1,0,1)
         SORTH_WEST=(1,-1,np.sqrt(2))
         SORTH_EASR=(1,1,np.sqrt(2))
         NORTH_WEST=(-1,1,np.sqrt(2))
         NORTH_EAST=(-1,-1,np.sqrt(2))```
         
 * add the following code into the `valid_action` 
     ```  
     if x-1<0 or grid[x-1,y]==1:
     valid_actions.remove(Action.NORTH)
                                   
 * plan_path
    
   `def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        self.target_position[2] = TARGET_ALTITUDE` 

  
 * set the global home position
  
   - read the first line from the file'colliders.csv'
   - and alter lat0, lon0  into floating point values
  
        `with open('colliders.csv', 'r') as f:
               latlon = f.readline()
           ll = latlon.strip().replace(',', '').split(' ')
           lat0, lon0 = float(ll[1]), float(ll[3])
           self.set_home_position(lon0, lat0, 0)`
           
           
* set the local positon to start position
     - retrieve current global position
     -use the global_to_local to  convert to local position 
         
         global_position=[self._latitude,self._longitude,self._altitude]
         
        
          local_position=global_to_local(self.global_position,self.global_home)
          print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
                                                                         
  * convert start position to current position
    - get the gird and the start offset
    
       grid_start1 = (-north_offset, -east_offset)
       grid_start = (int(local_position[0] + grid_start1[0]), int(local_position[1] + grid_start1[1])) `
    
   ### Step 9 : run the motion_planning
     >1.run the simulator
   
     >2.activate fcnd2 run `python my motionplanning.py`
   
  


