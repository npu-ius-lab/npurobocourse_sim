# tams_apriltags: 3D models with Apriltag textures.

This ROS package provides URDF models of thin cards with Apriltag textures, 
plus some example objects with attached Apriltag markers.
Adding a few markers to your robots or enviroments allows you 
to test full vision pipelines based on the Apriltags detector.

All markers of the 16h5 and 36h11 tag families are included;
markers of the other families can be generated quickly if needed.
The implementation uses a set of 3D collada meshes,
each one with the corresponding marker image as its texture.
This should work in all recent versions of Gazebo and in rviz, 
using ROS Kinetic or newer.

## Example

A Pioneer and some AprilCubes in Gazebo:

[Pioneer robot and AprilCubes in Gazebo](Media/surface/gazebo-aprilcubes.png)

The AprilCubes in rviz:

[AprilCubes in rviz](Media/surface/rviz-aprilcubes.png)


## Installation

Clone this repository into your workspace and run `catkin_make`.
There are no nodes to build.

## Usage

For a first selftest using Gazebo and rviz for visualization:
```
roslaunch tams_apriltags apriltag_demo.launch
```

See the provided `april_cube.xacro` file for an example on how
to attach (multiple) apriltags to an URDF robot model:
```
roslaunch tams_apriltags april_cube_demo.launch
```

See `apriltag_marker.xacro` for the actual Xacro macro and its parameters.
You can specify marker name, tag family, tag ID, and the marker size by using parameters
"size" and "apriltag_size". In case of specifying both size parameters, the "size" parameter 
will be recalculated. At the moment, the macro requires a parent object and generates 
the corresponding fixed joint.


Note the use of Python expressions (introduced in ROS Jade)
to rewrite the tag family string and to ensure leading zeros for the tag ID.
The slightly weird scaling is needed because the master Collada mesh
has size (100 x 100 x 0.1) millimeters.


## Notes

At the moment, only the 16h5 and 36h11 tag families are provided.
Don't hesitate to edit and run the provided generation scripts
to create additional markers.

