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
 
 
## 3.Build on ROS


## 5.Usage


## 6.Acknowledgements


## 7.Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.


## 8.Notes
- I would complete the readme soon.
- The code is written for research purpose.
