## SingleRobot-MultipleRobot
### You can find detail tutorial in https://link.medium.com/kZFB4aVqVX
1-Single Robot:
The warehouse is composed of grid cells. There can be obstacles for the robot in the warehouse.You may think obstacles as separators forming compartments in the warehouse or other machines like conveyor belts etc. You have one robot occupying a cell in the warehouse and it needs to go to a target cell to fetch a product or process a good in a machine. The robot can move up, down,left, and right (no direct diagonal movement). Additionally, the robot may wait at a time step.The robot must not hit either the walls of the warehouse or the obstacles in the warehouse. It should navigate to its target as quickly as possible.
### To this aim A* search as implemented with Manhattan distance.
2-Multiple Robot:
The Diffrence is not just one robot navigate,control many robot.To this aim robots will move one by one according to alphabetic. Alongside,the algorithm have to check previous robot path and make sure that there is no collisions.It other words, in any time two robots can no stand same cell.
