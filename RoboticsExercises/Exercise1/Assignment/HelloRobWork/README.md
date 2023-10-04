# HelloWorldRobWork

This tutorial will show you how to make a RobWork project in the terminal and how to import the project to QtCreator.
 The HelloWorldRobWork program will show you how to create a:

- **Joint Configuration:** The First number is the number of robot joints (also called the Number of Degrees of Freedom, NDOF) and the rest of the coordinates are the angles of the joints in radians.
- **Transform3D** has a postion and rotational part and can be constructed in different ways.
- **Equivalent Angle Axis (EAA):** The direction vector should have a length of 1.

Go through the program code.

Before starting take a look at the RobWork website. Here you can get more information about RobWork:

- RobWork website: http://www.robwork.dk/
- RobWork manual: http://www.robwork.dk/manual
- RobWork C++ API: http://www.robwork.dk/apidoc/cpp/doxygen/
- RobWork Python API: https://www.robwork.dk/apidoc/python/

Try searching for:

- Q and choose Q.
- Transform3D and choose Transform3D\<T>.
- EAA and choose EAA\<T>.

Here you can see the namespace and how to include it.

## Terminal

Open a new terminal by either:

- Click on the terminal icon in the menu-bar on the left side.
- Press Ctrl+Alt+t

Go to the build folder in the workspace:

``` bash
cd $\sim$/PathToFile/HelloWorldRobWork
mkdir build
cd build
```

Run CMake on the project:

> cmake ..

Make the project:
> make

The executable has now been placed in the build folder and can be run with:

> ./HelloWorldRobWork

## Python

Open a new terminal as described above, and run use:
> python3 $\sim$/PathToFile/HelloWorldRobWork/HelloWorldRobWork.py
