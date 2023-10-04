# Exercise 1

## Part 1 (HalloRobWork)

- Download and open this folder in Visual Studio Code.
- Open a terminal by going to the ToolBar select Terminal->new Terminal.
- navigate to HalloRobWork in terminal using "cd"
- create a new folder called build "mkdir build"
- go to the build folder
- call "cmake .."
- call "make"
- The Project has now been build
- Execute the Program with "./HelloRobWork"

## Part 2 (RobWorkStudio Intro)

- open Terminal
- call "RobworkStudio" this will start RobWorkStudio
- in RobWorkStudio open Kuka16WC/Scene.wc.xml
- find the Jog module in the ToolBar and open it.
- Use it to Move the robot over the Bottle.
  - Using only Joint position
  - Using invKin
- find the TreeView module in the ToolBar and open it.
- Using TreeView show the Frame and label of the TCP and Bottle
  - HINT: Right click on the frame, to get optins
- Allign the TCP Frame and the Bottle Frame so they Perfectly overlap
  - How many different Joint configurations can you find that does this


## Part 3 (Assemple a UR in RobWork)

- in the Folder UR5WorkCellCut you'll find a partial WorkCell for the UR5
- Finish the WorkCell by inserting the missing Joints, and geometries.
  - HINT: Check the UR-6-85-5-A.pdf for a technical drawing of the ur
  - HINT: Check <https://robwork.dk/file_formats/workcell/> for how to work with the WorkCell file
