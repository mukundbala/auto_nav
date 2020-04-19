# Introduction 
The NUS module EG2310, taken by year 1 iDP students introduces freshmen students to the basic principles of systems design that involve mechanical, electrical and software elements. Using a Robotis TurtleBot 3 Burger, students are taught basic theory of system design in the first half of the semester, with the second half devoted to application of materiel taught. 

# The Objective
To navigate and map an unknown area no larger than 8m x 8m, returning the mapped data as an image. 
To locate and fire at a target, no furhter than 2m away, identified as one of the R,G,B paper targets on the wall of the maze. 
Mapping is the primary objective, with target location and firing the secondary objective. 

# Autonomous Navigation
Autonomous Navigation is achieved by using occupancy data to find the next accessible unmapped region and sending the coordinates to the navigation stack which will move the robot to the specified coordinates. The unmapped region is found by using OpenCV image manipulation processes on the occupancy data. Once there are no more unmapped regions, the robot will stop and save the map and time taken. 
