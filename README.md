<h1><b>ROS2 CPP Mini-Project</b></h1>

<hr>

<h2>Avoiding Obstacles with VFF</h2>
<h3>Introduction</h3>
<h4>In this repository, a turtle bot is programmed to navigate and avoid obstacles in simulation (Gazebo) using the <b>Vitual Force Field (VFF)</b> algorithm.</h4>
<h4>The VFF algorithm is a local navigation method used when a robot follows a global plan to move from point A to point B, while dynamically avoiding static obstacles in its environment. It relies on three 2D vectors to compute the speed commands that help the robot steer away from obstacles:</h4>
<ol>
<li><b>Attractive Vector:</b> In our application this vector always points forward, in the x-direction of the robot, assuming there are no obstacles in the path. In real-world applications, it typically points toward the next waypoint on the robot’s planned path.</li>
<li><b>Repulsive Vector:</b> This vector is calculated based on readings from the laser sensor (e.g., LIDAR). You are encouraged to experiment with different methods for computing this vector. A simple approach is to use the closest detected obstacle and generate a repulsive force that is inversely proportional to its distance from the robot.</li>
<li><b>Resultant Vector:</b> This is the sum of the Attractive and Repulsive vectors. It determines the final control output:</li>
<ul>
<li>Linear speed is proportional to the magnitude of the Resultant vector.</li>
<li>Angular speed is proportional to the angle or direction of the Resultant vector.</li>
</ul>
</ol>
<hr>
<h3>ROS2 Package Structure</h3>

```
vff_avoidance/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── AvoidanceNodeConfig.yaml
├── include/
│   └── vff_avoidance/
│       └── AvoidanceNode.hpp
├── launch/
│   └── avoidance_vff.launch.py
└── src/
    ├── avoidance_vff_main.cpp
    └── vff_avoidance/
        └── AvoidanceNode.cpp
```


<ul><li>Note that in this implementation, the <b><code>vff_avoidance</code></b> package is inside the following directory: <b><code>/avoidance_node/src</code></b></li></ul>

<hr>

<h3>Documentation of Files</h3>
<ul>
<li><b><code>AvoidanceNodeConfig.yaml</code></b>:</li>
<li><b><code>AvoidanceNode.hpp</code></b>:</li>
<li><b><code>avoidance_vff.launch.py</code></b>:</li>
<li><b><code>avoidance_vff_main.cpp</code></b>:</li>
<li><b><code>AvoidanceNode.cpp</code></b>:</li>
<li><b><code>CMakeLists.txt</code></b>:</li>
<li><b><code>package.xml</code></b>: Tells ROS2 the package name, the dependencies, the maintainer name, version, description. This file is used when we are building the package using <b><code>colcon build</code></b>, in fact ROS2 read this file when building the package to know which packages and dependencies to use.</li>
</ul>