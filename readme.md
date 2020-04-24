# rustros_tf

This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of 
multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.

## Current Progress
Currently, this is a WIP. So far the only the following have been implemented:
* tfBuffer::lookup within same time frame

I am still working on the following:
* time traversal
* documentation
* freeze API
