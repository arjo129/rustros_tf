# rustros_tf

This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.

## Features
So far the only the following have been implemented:
* `TfListener` with `lookup_transform` and time traversal. 

I am still working on the following:
* Integration with point clouds.
* Integration with image geometry.
* Removal of `ndarray` as a dependency.
* Adding `nalgebra` related conversion methods. 
* More efficient cache data structure.
* A `TfBroadcaster` struct.

## Supported platforms
Currently only Ubuntu 18.04 running ROS Melodic on x86_64 is working. It should work on any linux based system with a proper ROS installation.

## Getting Started
Install [ROS](http://wiki.ros.org/melodic/Installation) first. On ubuntu, this can be done like so:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop
```
You will also need a fortran compiler.
```
sudo apt install gfortran
```
After installing ROS, you may simply add this crate as a dependency to your cargo project:
```
[dependencies]
rustros_tf = "0.1.0"
```

# Example usage
The following example shows a simple lookup. 
```
extern crate rosrust;
extern crate rosrust_msg;
extern crate rustros_tf;

use rustros_tf::TfListener;

fn main() {
    rosrust::init("listener");
    let listener = TfListener::new();
    
    let rate = rosrust::rate(1.0);
    while rosrust::is_ok() {
        let tf = listener.lookup_transform("camera", "base_link", rosrust::Time::new());
        println!("{:?}", tf);
        rate.sleep();
    }
}
```