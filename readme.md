
# RoboCup RRT 3dprint

A 3d-printed model showing RoboCup SSL path planning using an rrt.

This model is generated by a C++ program that uses the [RoboJackets RRT library](https://github.com/robojackets/rrt) to find a path from a given start location to a desired destination, avoiding collisions with other robots.
The model shows the all areas explored by the rrt trees and draws the final path in "bold".
This C++ program generates an openscad file ('out.scad') listing the coordinates of all of the segments in the tree as well as the positions of the robots.
The 'main.scad' file imports 'out.scad' and uses the included data to draw a 3d model of the scene using openscad.
This 'main.scad' model can be opened in openscad to preview and generate an stl file suitable for import into a 3d printer slicing program.

## Preview

Check out the 3d preview [here](doc/963dec.stl) or see the screenshot below.

![screenshot](doc/screenshot.png)


## Customizing

You can easily edit these files to customize the display or layout of the model.

'main.scad'

* rrt segment 3d sizing
* node shape, size
* robot details (these could use some improvement)
* field details

'main.cpp'

* robot positions, size
* field size
* rrt parameters
* rrt random seed
