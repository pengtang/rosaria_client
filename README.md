This is the rosaria client

*******************************************ROSARIA CLIENT USAGE*****************************************************

Prerequisite: catkin workspace is setup, and use "git clone https://github.com/pengtang/rosaria_client.git" under the directory of catkin_ws/src. Then, under catkin_ws directory type "catkin_make" to compile and make executable files. Last, still under catkin_ws, type "source devel/setup.bash".

Step 1. If you are operating with a real robot, look at Case 1, if you are using the mobilesim, look at Case 2.

  Case 1: Please refer to http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA
  
  Case 2: Open mobilesim

Step 2. Open a terminal with three tabs. Do this sequentially: The first tab type "roscore", the second tab type "rosrun rosaria RosAria", the third tab type "rosrun rosaria_client interface"

Step 3. Follow the instruction in the interface program, run the program respectively.

Notice: launch file will be added shortly. 

Update date: Apr. 18
*************************************************************************

Beta V1.0 Most of the time it works fine, sometimes it does not stop(continue going ahead or continue spinning).

Beta V1.1 (Mar. 28) Fix the bug sometimes the robot does not stop Add feature of Stopping the robot first, and then do its functionality

Beta V1.2 (Apr. 5) Add print_state function(could print part of the info right now, need to be enhanced).

Beta V1.3 (Apr. 10) Pushed some small corrections to spin_clockwise, renamed spin_anticlockwise to spin_counterclockwise and added small corrections to that as well. Also added (very) rough draft of interface program.

Beta V1.4 (Apr. 10) Change the spin from 180 to 90 degree, finished the print_state function.

Beta V1.5 (Apr. 14) Add function enable_motors, fix the printing information about the front_motors and rear_motors state.

Beta V1.6 (Apr 16) Added a functional version of the interface 

Beta V1.7 (Apr 18) Changed stop key to spacebar in teleop, where the user now hits 'q' to quit.  Also edited interface to prompt user for inputs every time its ready for a selection

Beta V1.8 (Apr. 20) Add launch file for rosaria_client
