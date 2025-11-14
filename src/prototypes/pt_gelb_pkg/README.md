# This is the code to run our first prototype named 'gelb'
**Prerequisites**
This packages run on ubuntu 24.4 and ROS2 jazzy jalisco and is custom made for a Raspberry Pi4

**Docker Containter**
Run the Dockerfile "Dockerfile" in src/prototypes/pt_gelb_pkg and mount your code or run "Dockerfile.base" in the same directory and the code will be launched on it's own (baked in).

**Custom ROS2 nodes**
It contains two custom nodes:
"thrust_control_fw.py" which subscribes to the joy node that publishes PS4-controller readings and maps the R2-button-inputs to PWM signals and writes them on GPIO-pin-18 

"thrust_control_fwbw.py" which subscribes to the joy node that publishes PS4-controller readings and maps the L2-button- and R2-button-inputs to forward and backward PWM signals and writes them on GPIO-pin-18.

**Colcon Build**
Ensure the custom packages are built before trying to run them. Before building it remove the COLCON_IGNORE file. Then build the packages with the following bash command in the root directory of this repo.
```
colcon build --packages-select pt_gelb_pkg
```

**Launch code**
The launch file "gelb_launch.py" launches the joy_node from package joy and the thrust_control_fwbw from package pt_gelb_pkg.

with ```ros2 launch pt_gelb_pkg gelb_launch``` you'll run it.