# pointcloudTraj

## 1.Introduction

We present a framework for online generating safe and dynamically feasible trajectories **directly**
on the point cloud, which is the lowest level representation of range measurements and is applicable to different sensor types. We online generate and refine a flight corridor which represents the free space that the trajectory of the quadrotor should lie in. We represent the trajectory as piecewise Bézier curves by using the Bernstein polynomial basis and formulate the trajectory generation problem as a convex program. By using Bézier curves, we can constrain the position and kinodynamics of the trajectory entirely within the flight corridor and given physical limits. For details we refer readers to our paper.

**Authors:**[Fei Gao](https://ustfei.com/) and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HUKST Aerial Robotics Group](uav.ust.hk).

**Disclaimer**

This is research code, any fitness for a particular purpose is disclaimed.

**Related Paper**
* **Flying on point clouds: Online trajectory generation and autonomous navigation for quadrotors in cluttered environments**, Fei Gao, William Wu, Wenliang Gao, Shaojie Shen, **Journal of Field Robotics** (2018).

Video of this paper can be found at:

<a href="https://www.youtube.com/watch?v=b9F2x3R6ri8&t=2s" target="_blank"><img src="https://img.youtube.com/vi/b9F2x3R6ri8/0.jpg" 
alt="video" width="752" height="480" border="10" /></a>

If you use this planning framework for your academic research, please cite our related paper.
```
@article{gao2018flying,
  title={Flying on point clouds: Online trajectory generation and autonomous navigation for quadrotors in cluttered environments},
  author={Gao, Fei and Wu, William and Gao, Wenliang and Shen, Shaojie},
  journal={Journal of Field Robotics},
  year={2018},
  publisher={Wiley Online Library}
}
```
## 2.Prerequisities
- Our testing environment: **Ubuntu** 16.04, **ROS** Kinetic.
- We use **mosek** for solving quadratic program(QP). To use mosek, you should approve an academic license in [here](https://www.mosek.com/products/academic-licenses/). The academic license is free and is easy to approve. Then create a folder named 'mosek' in your home directory and put your license in it. All header and library files are already included in this repo, so you don't need to download mosek again. 
- The package 'odom_visualization' depends on [armadillo](http://arma.sourceforge.net/), which is a c++ linear algebra library. This package is not necessary and only used for visualization. You can install 'armadillo' by:

```
  sudo apt-get install libarmadillo-dev 
```
 
## 3.Build on ROS
  Clone the repository to your catkin workspace and catkin_make. For example:
```
  cd ~/catkin_ws/src
  git clone https://github.com/HKUST-Aerial-Robotics/pointcloudTraj.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```

## 5.Usage
- Use the command following to run a demo of the drone's planning. A target coordinate is given in the launch file.
```
roslaunch pointcloudTraj clean_demo.launch
```

- If you want to try an interactive way to send commands to the drone, you have to install a 'rviz_plugin' from [plan_utils](https://github.com/HKUST-Aerial-Robotics/plan_utils). Then launch the simulation.launch:
```
roslaunch pointcloudTraj simulation.launch
```

After that, in rviz, click 'Panels -> tools -> +' and select the plugin 'Goal3DTool'. If you have successfully compiled all packages from [plan_utils](https://github.com/HKUST-Aerial-Robotics/plan_utils), now you can see *3D Nav Goal* in the tools panel.

We use *3D Nav Goal* to send a target for the drone to navigate. To use it, click the tool (shortcut keyboard 'g' may conflict with *2D Nav Goal*), then press on left mouse button on a position in rviz, click right mouse button to start to drag it slide up or down for a targeting height (don't loose left button at this time). Finally you loose left mouse button and a target will be sent to the planner, done.
 solving second-order cone program(SOCP).

## 6.Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.


## 7.Notes
- I would complete the readme soon.
- The code is written for research purpose.
