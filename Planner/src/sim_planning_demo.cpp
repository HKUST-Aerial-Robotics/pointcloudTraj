#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>

#include "pointcloudTraj/mosek.h"
#include "pointcloudTraj/backward.hpp"
#include "pointcloudTraj/data_type.h"
#include "pointcloudTraj/bezier_base.h"
#include "pointcloudTraj/corridor_finder.h"
#include "pointcloudTraj/traj_optimizer.h"

namespace backward {
backward::SignalHandling sh;
}

using namespace std;
using namespace Eigen;
using namespace pcl;

ros::Publisher _vis_corridor_pub;
ros::Publisher _vis_rrt_star_pub;
ros::Publisher _vis_ctrl_pts_pub;
ros::Publisher _vis_commit_target_pub;
ros::Publisher _traj_pub;
ros::Publisher _vis_target_points;
ros::Publisher _vis_traj_points;
ros::Publisher _vis_commit_traj_points;
ros::Publisher _vis_stop_traj_points;
ros::Subscriber _odom_sub;
ros::Subscriber _dest_pts_sub;
ros::Subscriber _map_sub;
ros::Timer _planning_timer;

/*  parameters read from lauch file  */
double _vel_max, _acc_max, _vel_mean, _acc_mean, _eps;
double _x_l, _x_h, _y_l, _y_h, _z_l, _z_h;  // For random map simulation : map boundary
double _refine_portion, _path_find_limit, _sample_portion, _goal_portion;
double _safety_margin, _search_margin, _max_radius, _sensing_range, _planning_rate, _stop_time, _time_commit;
int    _minimize_order, _poly_order_min, _poly_order_max, _max_samples;
bool   _use_preset_goal, _is_limit_vel, _is_limit_acc, _is_print;

/*  useful global variables  */
Vector3d _start_pos, _start_vel, _start_acc, _commit_target, _end_pos;      
double _time_limit_1, _time_limit_2;
nav_msgs::Odometry _odom;

int _traj_id = 0;
int _segment_num;
vector<int> _poly_orderList;
ros::Time _odom_time;
ros::Time _start_time     = ros::TIME_MAX;
ros::Time _rcv_odom_time  = ros::TIME_MAX;
ros::Time _plan_traj_time = ros::TIME_MAX;

bool _is_traj_exist     = false;
bool _is_target_arrive  = false;
bool _is_target_receive = false;
bool _is_has_map        = false;

MatrixXd _Path;
MatrixXd _PolyCoeff;
VectorXd _Time;
VectorXd _Radius;

vector<MatrixXd> _FMList;
vector<VectorXd> _CList, _CvList, _CaList;
sensor_msgs::PointCloud2 traj_pts, target_pts, traj_commit_pts, traj_stop_pts;
PointCloud<PointXYZ> traj_pts_pcd, target_pts_pcd, traj_commit_pts_pcd, traj_stop_pts_pcd;
visualization_msgs::MarkerArray path_vis;
visualization_msgs::MarkerArray ctrlPt_vis;
safeRegionRrtStar _rrtPathPlaner;
TrajectoryOptimizerSOCP _trajectoryGeneratorSocp;

bool checkSafeTrajectory(double check_time);
int trajGeneration(MatrixXd path, VectorXd radius, double time_odom_delay);
Vector3d getCommitedTarget();
VectorXd timeAllocation(MatrixXd sphere_centers, VectorXd sphere_radius, Vector3d init_vel );
quadrotor_msgs::PolynomialTrajectory getBezierTraj();

void visCommitTraj(MatrixXd polyCoeff);
void visBezierTrajectory(MatrixXd polyCoeff);
void visCtrlPoint(MatrixXd polyCoeff);
void visFlightCorridor(MatrixXd path, VectorXd radius);
void visRrt(vector<NodePtr> nodes);

void getPosFromBezier(const MatrixXd & polyCoeff,  const double & t_now, const int & seg_now, Vector3d & ret);
void getStateFromBezier(const MatrixXd & polyCoeff,  const double & t_now, const int & seg_now, VectorXd & ret);

void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    if(wp.poses[0].pose.position.z < 0.0)
      return;

    _end_pos(0) = wp.poses[0].pose.position.x;
    _end_pos(1) = wp.poses[0].pose.position.y;
    _end_pos(2) = wp.poses[0].pose.position.z;

    _is_target_receive  = true;
    _is_target_arrive   = false;
    _is_traj_exist      = false;
}

void rcvOdometryCallBack(const nav_msgs::Odometry odom)
{
    _odom = odom;
    _odom_time = odom.header.stamp;

    _start_pos(0) = _odom.pose.pose.position.x;
    _start_pos(1) = _odom.pose.pose.position.y;
    _start_pos(2) = _odom.pose.pose.position.z;
    _start_vel(0) = _odom.twist.twist.linear.x;
    _start_vel(1) = _odom.twist.twist.linear.y;
    _start_vel(2) = _odom.twist.twist.linear.z;
    _start_acc(0) = _odom.twist.twist.angular.x;
    _start_acc(1) = _odom.twist.twist.angular.y;
    _start_acc(2) = _odom.twist.twist.angular.z;

    _rcv_odom_time = ros::Time::now();
    _rrtPathPlaner.setStartPt(_start_pos, _end_pos);  

    const static auto getDis = []( const Vector3d u, const Vector3d v ){
        const static auto sq = [] (double && val){ 
            return val * val;
        };

        return sqrt( sq(v[0] - u[0]) + sq(v[1] - u[1]) + sq(v[2] - u[2])  );
    };

    if( _is_traj_exist && getDis(_start_pos, _commit_target) < _eps )
        _is_target_arrive = true;
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{   
    PointCloud<PointXYZ> cloud_input;      
    pcl::fromROSMsg(pointcloud_map, cloud_input);

    if(cloud_input.points.size() == 0) return;

    _is_has_map = true;
    _rrtPathPlaner.setInput(cloud_input);
    if(checkSafeTrajectory(_stop_time))
    {
        ROS_WARN("[Demo] Collision Occur, Stop");
        quadrotor_msgs::PolynomialTrajectory traj;
        traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
        _traj_pub.publish(traj);
        _is_traj_exist = false;  
    }
}

void corridorAugment(MatrixXd & path, VectorXd & radius, VectorXd & time)
{   
    int seg_num = time.size();
  
    MatrixXd path_(seg_num + 1, 3);
    VectorXd radi_(seg_num + 1 );
    VectorXd time_(seg_num + 1 );

    path_.row(0) = path.row(0);
    path_.block(1, 0, seg_num, 3) = path;

    radi_(0) = radius(0);
    radi_.segment(1, seg_num) = radius;

    time_(0) = 0.5;
    time_.segment(1, seg_num) = time;
    time_(1) -= 0.5;

    path   = path_;
    radius = radi_;
    time   = time_; 
}

int trajGeneration(MatrixXd path, VectorXd radius, double time_odom_delay)
{           
    MatrixXd pos(2,3), vel(2,3), acc(2,3);

    if(_is_traj_exist){
        double time_est_opt = 0.03;      
        double t_s =  (_odom_time - _start_time).toSec() + time_est_opt + time_odom_delay;     
        int idx;
        for (idx = 0; idx < _segment_num; ++idx){
            if (t_s > _Time(idx) && idx + 1 < _segment_num)
                t_s -= _Time(idx); 
            else break;
        }

        t_s /= _Time(idx);

        VectorXd state;
        getStateFromBezier(_PolyCoeff, t_s, idx, state);

        for(int i = 0; i < 3; i++ ){
            pos(0, i) = state(i) * _Time(idx);
            vel(0, i) = state(i + 3); 
            acc(0, i) = state(i + 6) / _Time(idx); 
        }  
    }
    else{   
        pos.row(0) = _start_pos;
        vel.row(0) = _start_vel;
        acc.row(0) = _start_acc;
    }

    pos.row(1) = _end_pos;
    vel.row(1) = VectorXd::Zero(3);
    acc.row(1) = VectorXd::Zero(3);

    _Time = timeAllocation( path, radius, vel.row(0) );
    corridorAugment(path, radius, _Time);
    _segment_num = radius.size();

    // Now we assign a order to each piece od the trajectory according to its size
    auto min_r = radius.minCoeff();
    auto max_r = radius.maxCoeff();

    _poly_orderList.clear();
    double d_order = (_poly_order_max - _poly_order_min ) / (max_r - min_r);
    
    for(int i =0; i < _segment_num; i ++ )
    {  
        if(i == 0 || i == _segment_num - 1) _poly_orderList.push_back( _poly_order_max );
        else _poly_orderList.push_back( int( _poly_order_min + (radius(i) - min_r) * d_order ) );
    }

    ros::Time time_1 = ros::Time::now();
    
    _PolyCoeff = _trajectoryGeneratorSocp.BezierConicOptimizer(  
                path, radius, _Time, _poly_orderList, _FMList, pos, vel, acc, _vel_max, _acc_max, _minimize_order, _is_limit_vel, _is_limit_acc, _is_print);
    
    _start_time = _odom_time + ros::Duration(ros::Time::now() - _rcv_odom_time);
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("[Demo] Time in traj generation is %f", (time_2 - time_1).toSec());
    
    quadrotor_msgs::PolynomialTrajectory traj;
    if(_PolyCoeff.rows() == 3 && _PolyCoeff.cols() == 3){
        ROS_WARN("[Demo] Cannot find a feasible and optimal solution, somthing wrong with the mosek solver ... ");
        traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
        _traj_pub.publish(traj);
        _is_traj_exist     = false;
        _is_target_receive = false;
        return -1;
    }
    else{
        ROS_WARN("[Demo] Trajectory generated successed");
        traj = getBezierTraj();
        _traj_pub.publish(traj);
        _is_traj_exist = true;
    }

    _traj_id ++;
    return 1;
}

bool checkEndOfCommitedTraj()
{     
    if(_is_target_arrive){     
      _is_target_arrive = false;
      return true;
    }
    else return false;
}

Eigen::Vector3d getCommitedTarget()
{
    double t_s = _time_commit;      

    int idx;
    for (idx = 0; idx < _segment_num; ++idx){
        if (t_s > _Time(idx) && idx + 1 < _segment_num)
            t_s -= _Time(idx);
        else break;
    }          
    t_s /= _Time(idx);

    double t_start = t_s;
    Eigen::Vector3d coord_t;
    getPosFromBezier(_PolyCoeff, t_start, idx, coord_t);

    coord_t *= _Time(idx);
    target_pts_pcd.clear();
    target_pts_pcd.points.push_back(PointXYZ(coord_t(0), coord_t(1), coord_t(2)));    
    target_pts_pcd.width = target_pts_pcd.points.size();
    target_pts_pcd.height = 1;
    target_pts_pcd.is_dense = true;

    pcl::toROSMsg(target_pts_pcd, target_pts);
    target_pts.header.frame_id = "map";
    _vis_target_points.publish(target_pts);

    return coord_t;
}

void planInitialTraj()
{
    _rrtPathPlaner.reset();
    _rrtPathPlaner.setPt( _start_pos, _end_pos, _x_l, _x_h, _y_l, _y_h, _z_l, _z_h, _sensing_range, _max_samples, _sample_portion, _goal_portion );

    ros::Time timeBef = ros::Time::now();
    _rrtPathPlaner.SafeRegionExpansion(_path_find_limit);
    ros::Time timeAft = ros::Time::now();
    double _path_time = (timeAft-timeBef).toSec();

    tie(_Path, _Radius) = _rrtPathPlaner.getPath();
    if( _rrtPathPlaner.getPathExistStatus() == false ){
        ROS_WARN("[Demo] Can't find a path, mission stall, please reset the target");
        quadrotor_msgs::PolynomialTrajectory traj;
        traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
        _traj_pub.publish(traj);
        _is_traj_exist = false;
        _is_target_receive = false;
    }
    else{ // Path finding succeed .
        if( trajGeneration( _Path, _Radius, _path_time ) == 1 ){   
            _commit_target = getCommitedTarget();
            _rrtPathPlaner.resetRoot(_commit_target);
            visBezierTrajectory(_PolyCoeff);
            visCommitTraj(_PolyCoeff);
            visCtrlPoint( _PolyCoeff );
            visFlightCorridor(_Path, _Radius);
        }
    }

    visRrt(_rrtPathPlaner.getTree());
}

void planIncrementalTraj()
{     
    if(_rrtPathPlaner.getGlobalNaviStatus() == true){
      visRrt(_rrtPathPlaner.getTree());
      return;
    }

    if( checkEndOfCommitedTraj() ) {   // arrive at the end of the commited trajectory.
        if( !_rrtPathPlaner.getPathExistStatus() ){ // no feasible path exists
            ROS_WARN("[Demo] reach commited target but no path exists, waiting for a path");
            _is_traj_exist = false;  // continue to fly 
            return;
        }
        else{   
            visFlightCorridor( _Path, _Radius);
            _plan_traj_time = ros::Time::now();
            if( trajGeneration( _Path, _Radius, (_plan_traj_time - _rcv_odom_time).toSec() ) == 1) { // Generate a new trajectory sucessfully.
                _commit_target = getCommitedTarget();   
                _rrtPathPlaner.resetRoot(_commit_target);  

                ros::Time time_1 = ros::Time::now();
                visBezierTrajectory(_PolyCoeff);
                visCommitTraj(_PolyCoeff);
                visCtrlPoint (_PolyCoeff);
            }
        }
    }
    else{   // continue to refine the uncommited trajectory
        _rrtPathPlaner.SafeRegionRefine  ( _time_limit_1 ); // add samples to the tree
        _rrtPathPlaner.SafeRegionEvaluate( _time_limit_2 ); // ensure that the path is collision-free

        if(_rrtPathPlaner.getPathExistStatus() == true){ 
            tie(_Path, _Radius) = _rrtPathPlaner.getPath();
            visFlightCorridor(_Path, _Radius);
        }

        visRrt(_rrtPathPlaner.getTree()); 
    }
}

void planningCallBack(const ros::TimerEvent& event)
{
    if( !_is_target_receive || !_is_has_map ) 
        return;

    if( !_is_traj_exist ) planInitialTraj();
    else planIncrementalTraj();  
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "point_cloud_planer_node");
    ros::NodeHandle node_handle( "~" );

    node_handle.param("planParam/plan_rate",        _planning_rate,   10.0);      
    node_handle.param("planParam/safety_margin",    _safety_margin,   0.65);
    node_handle.param("planParam/search_margin",    _search_margin,   0.35);
    node_handle.param("planParam/max_radius",       _max_radius,      10.0);
    node_handle.param("planParam/sensing_range",    _sensing_range,   10.0);     
    node_handle.param("planParam/refine_portion",   _refine_portion,  0.80);     
    node_handle.param("planParam/sample_portion",   _sample_portion,  0.25);     // the ratio to generate samples inside the map range
    node_handle.param("planParam/goal_portion",     _goal_portion,    0.05);     // the ratio to generate samples on the goal
    node_handle.param("planParam/path_find_limit",  _path_find_limit, 0.05);     
    node_handle.param("planParam/max_samples",      _max_samples,     3000);     
    node_handle.param("planParam/stop_horizon",     _stop_time,       0.50);     
    node_handle.param("planParam/commitTime",       _time_commit,      1.0);

    node_handle.param("dynamic/vec",       _vel_mean, 2.0);
    node_handle.param("dynamic/acc",       _acc_mean, 1.0);
    node_handle.param("dynamic/max_vec",   _vel_max,  3.0);
    node_handle.param("dynamic/max_acc",   _acc_max,  1.5);

    node_handle.param("mapBoundary/lower_x", _x_l,  -50.0);
    node_handle.param("mapBoundary/upper_x", _x_h,   50.0);
    node_handle.param("mapBoundary/lower_y", _y_l,  -50.0);
    node_handle.param("mapBoundary/upper_y", _y_h,   50.0);
    node_handle.param("mapBoundary/lower_z", _z_l,    0.0);
    node_handle.param("mapBoundary/upper_z", _z_h,    3.0);
    
    node_handle.param("optimization/poly_order_min", _poly_order_min,  5);
    node_handle.param("optimization/poly_order_max", _poly_order_max, 10);
    node_handle.param("optimization/minimize_order", _minimize_order,  3);
    
    node_handle.param("demoParam/target_x",     _end_pos(0),       0.0);
    node_handle.param("demoParam/target_y",     _end_pos(1),       0.0);
    node_handle.param("demoParam/target_z",     _end_pos(2),       0.0);
    node_handle.param("demoParam/goal_input",   _use_preset_goal, true);
    node_handle.param("demoParam/is_limit_vel", _is_limit_vel,    true);
    node_handle.param("demoParam/is_limit_acc", _is_limit_acc,    true);
    node_handle.param("demoParam/is_print",     _is_print,        true);

    Bernstein _bernstein;
    if(_bernstein.setParam(_poly_order_min, _poly_order_max, _minimize_order) == -1){
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
        exit(EXIT_FAILURE);
    }

    _FMList  = _bernstein.getFM();
    _CList   = _bernstein.getC();
    _CvList  = _bernstein.getC_v();
    _CaList  = _bernstein.getC_a();

    _eps = 0.25; 
    _rrtPathPlaner.setParam(_safety_margin, _search_margin, _max_radius, _sensing_range);

    _time_limit_1 =      _refine_portion  * 1.0 / _planning_rate;
    _time_limit_2 = (1 - _refine_portion) * 1.0 / _planning_rate; 
          
    // subcribed msgs
    _dest_pts_sub = node_handle.subscribe( "waypoints",  1, rcvWaypointsCallBack );
    _map_sub      = node_handle.subscribe( "PointCloud", 1, rcvPointCloudCallBack);
    _odom_sub     = node_handle.subscribe( "odometry",   1, rcvOdometryCallBack );

    /*  publish traj msgs to traj_server  */
    _traj_pub           = node_handle.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 10);
  
    /*  publish visualization msgs  */
    _vis_ctrl_pts_pub       = node_handle.advertise<visualization_msgs::MarkerArray>("trajectory_ctrl_pts", 1);
    _vis_corridor_pub       = node_handle.advertise<visualization_msgs::MarkerArray>("flight_corridor",     1);
    _vis_rrt_star_pub       = node_handle.advertise<visualization_msgs::Marker>     ("rrt_tree",            1);
    _vis_commit_target_pub  = node_handle.advertise<visualization_msgs::Marker>     ("commited_target",     1); 

    _vis_target_points      = node_handle.advertise<sensor_msgs::PointCloud2>("commit_target",              1);
    _vis_traj_points        = node_handle.advertise<sensor_msgs::PointCloud2>("trajectory_points",          1);
    _vis_commit_traj_points = node_handle.advertise<sensor_msgs::PointCloud2>("trajectory_commit_points",   1);
    _vis_stop_traj_points   = node_handle.advertise<sensor_msgs::PointCloud2>("trajectory_stop_points",     1);

    _planning_timer         = node_handle.createTimer(ros::Duration(1.0 / _planning_rate), planningCallBack);

    ros::Rate rate(100);

    if(_use_preset_goal)
    {   
        ROS_WARN("[Demo] Demo using preset target");
        ROS_WARN("[Demo] Warming up ... ");

        sleep(2.0);
        _is_target_receive  = true;
        _is_target_arrive   = false;
        _is_traj_exist      = false;

    }
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
}

quadrotor_msgs::PolynomialTrajectory getBezierTraj()
{
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj.num_segment = _Time.size();

    int polyTotalNum = 0;
    for(auto order:_poly_orderList)
        polyTotalNum += (order + 1);

    traj.coef_x.resize(polyTotalNum);
    traj.coef_y.resize(polyTotalNum);
    traj.coef_z.resize(polyTotalNum);

    int idx = 0;
    for(int i = 0; i < _segment_num; i++ )
    {    
        int order = _poly_orderList[i];
        int poly_num1d = order + 1;
        
        for(int j =0; j < poly_num1d; j++)
        { 
            traj.coef_x[idx] = _PolyCoeff(i,                  j);
            traj.coef_y[idx] = _PolyCoeff(i,     poly_num1d + j);
            traj.coef_z[idx] = _PolyCoeff(i, 2 * poly_num1d + j);
            idx++;
        }
    }

    traj.header.frame_id = "/bernstein";
    traj.header.stamp = _start_time;

    traj.time.resize(_Time.size());
    traj.order.resize(_Time.size());

    traj.mag_coeff = 1.0;

    for (int idx = 0; idx < _Time.size(); ++idx){
        traj.time[idx] = _Time(idx);
        traj.order[idx] = _poly_orderList[idx];
    }
    
    traj.start_yaw = 0.0;
    traj.final_yaw = 0.0;

    traj.trajectory_id = _traj_id;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

    return traj;
}

VectorXd timeAllocation( MatrixXd sphere_centers, VectorXd sphere_radius, Vector3d init_vel )
{   
    int ball_num = sphere_centers.rows();
    MatrixXd check_pt( ball_num - 1 , 3 );

    const static auto getDis = []( const Vector3d u, const Vector3d v ){
          const static auto sq = [] (double && val){ 
                return val * val;
            };

          return sqrt( sq(v[0] - u[0]) + sq(v[1] - u[1]) + sq(v[2] - u[2])  );
    };

    Vector3d center, last_center;
    double radius, last_radius;

    int index = 0 ;
    last_center <<  sphere_centers(index, 0), sphere_centers(index, 1), sphere_centers(index, 2); 
    last_radius  =  sphere_radius[index];

    for(index = 1; index < ball_num; index ++ ){   
        center <<  sphere_centers(index, 0), sphere_centers(index, 1), sphere_centers(index, 2); 
        radius  =  sphere_radius[index];

        double dist = getDis(last_center, center);  
        
        Vector3d delta_Vec = center - last_center;
        Eigen::Vector3d joint_pt;
        joint_pt = delta_Vec * ( dist + last_radius - radius) / ( 2.0 * dist ) + last_center; 

        check_pt.block( index - 1, 0, 1, 3 ) = joint_pt.transpose();

        last_center = center;
        last_radius = radius;
    }

    MatrixXd all_points( ball_num + 1 , 3 );
    all_points.row( 0 )                       = _start_pos;
    all_points.block( 1, 0, ball_num - 1, 3 ) =  check_pt;
    all_points.row( ball_num )                = _end_pos;

    VectorXd time_allocate(all_points.rows() - 1);
    Vector3d initv = init_vel;

    for (int k = 0; k < all_points.rows() - 1; k++){
        double dtxyz;
        Vector3d p0   = all_points.row(k);           // The start point of this segment
        Vector3d p1   = all_points.row(k + 1);       // The end point of this segment
        Vector3d d    = p1 - p0;                     // The position difference
        Vector3d v0(0.0, 0.0, 0.0);                  // The init velocity
        
        if( k == 0) v0 = initv;

        double D    = d.norm();                      // The norm of position difference 
        double V0   = v0.dot(d / D);                 // Map velocity to the vector (p1-p0)
        double aV0  = fabs(V0);                      // The absolote mapped velocity

        double acct = ( _vel_mean - V0) / _acc_mean * ( (_vel_mean > V0 ) ? 1 : -1 );            // The time to speed up to to the max veloctiy
        double accd = V0 * acct + (_acc_mean * acct * acct / 2) * ((_vel_mean > V0) ? 1 : -1 );  // The distance to speed up
        double dcct = _vel_mean / _acc_mean;                                                     // The time to slow down.
        double dccd = _acc_mean * dcct * dcct / 2;                                               // The distance to slow down.

        if (D < aV0 * aV0 / (2 * _acc_mean)){           // if not enough distance to slow down
            double t1 = (V0 < 0)?2.0 * aV0 / _acc_mean:0.0;
            double t2 = aV0 / _acc_mean;
            dtxyz     = t1 + t2;                 
        }
        else if (D < accd + dccd){                      // if not enough distance to speed up and slow dwon 
            double t1 = (V0 < 0)?2.0 * aV0 / _acc_mean:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _acc_mean * D - aV0 * aV0 / 2)) / _acc_mean;
            double t3 = (aV0 + _acc_mean * t2) / _acc_mean;
            dtxyz     = t1 + t2 + t3;    
        }
        else{                                          // can reach max veloctiy and keep for a while.
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _vel_mean;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
        }
        time_allocate(k) = dtxyz;
    }

    return time_allocate;
}

inline void getStateFromBezier(const MatrixXd & polyCoeff,  const double & t_now, const int & seg_now, VectorXd & ret)
{
    ret = VectorXd::Zero(9);
    VectorXd ctrl_now = polyCoeff.row(seg_now);

    int order = _poly_orderList[seg_now];
    int ctrl_num1D = order + 1;

    for(int i = 0; i < 3; i++)
    {   
        for(int j = 0; j < ctrl_num1D; j++){
            ret(i) += _CList[order](j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (order - j) ); 
            
            if(j < ctrl_num1D - 1 )
              ret(i+3) += _CvList[order](j) * order 
                        * ( ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j))
                        * pow(t_now, j) * pow((1 - t_now), (order - j - 1) ); 
            
            if(j < ctrl_num1D - 2 )
              ret(i+6) += _CaList[order](j) * order * (order - 1) 
                        * ( ctrl_now(i * ctrl_num1D + j + 2) - 2 * ctrl_now(i * ctrl_num1D + j + 1) + ctrl_now(i * ctrl_num1D + j))
                        * pow(t_now, j) * pow((1 - t_now), (order - j - 2) );                         
        }

    }
}

inline void getPosFromBezier(const MatrixXd & polyCoeff,  const double & t_now, const int & seg_now, Vector3d & ret)
{
    ret = VectorXd::Zero(3);

    int order = _poly_orderList[seg_now];
    int ctrl_num1D = order + 1;
    for(int i = 0; i < 3; i++)
    {   
        for(int j = 0; j < ctrl_num1D; j++){
            ret(i) += _CList[order](j) * polyCoeff(seg_now, i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (order - j) ); 
        }
    }
}

bool checkSafeTrajectory(double check_time)
{   
    if(!_is_traj_exist) return false;

    traj_stop_pts_pcd.points.clear();

    double t_s = max(0.0, (_odom.header.stamp - _start_time).toSec());      
    int idx;
    for (idx = 0; idx < _segment_num; ++idx){
      if (t_s > _Time(idx) && idx + 1 < _segment_num)
          t_s -= _Time(idx);
      else break;
    }
    
    double t_ss;
    double t_accu = 0.0;
    for(int i = idx; i < _segment_num; i++ ){
      t_ss = (i == idx) ? t_s : 0.0;
      for (double t = t_ss; t < _Time(i); t += 0.02){
        t_accu += 0.02;
        if(t_accu > _stop_time) break;

        Vector3d traj_pt;
        getPosFromBezier(_PolyCoeff, t/_Time(i), i, traj_pt);
        traj_pt *= _Time(i); 
        
        PointXYZ pt;
        pt.x = traj_pt(0);
        pt.y = traj_pt(1);
        pt.z = traj_pt(2);
        traj_stop_pts_pcd.points.push_back(pt);
              
        if(_rrtPathPlaner.checkTrajPtCol(traj_pt)){
          traj_stop_pts_pcd.width = traj_stop_pts_pcd.points.size();
          traj_stop_pts_pcd.height = 1;
          traj_stop_pts_pcd.is_dense = true;
          pcl::toROSMsg(traj_stop_pts_pcd, traj_stop_pts);
          traj_stop_pts.header.frame_id = "map";
          _vis_stop_traj_points.publish(traj_stop_pts);
          return true;
        }
      }
    }

    traj_stop_pts_pcd.width = traj_stop_pts_pcd.points.size();
    traj_stop_pts_pcd.height = 1;
    traj_stop_pts_pcd.is_dense = true;
    pcl::toROSMsg(traj_stop_pts_pcd, traj_stop_pts);
    traj_stop_pts.header.frame_id = "map";
    _vis_stop_traj_points.publish(traj_stop_pts);

    return false;
}

void visRrt(vector<NodePtr> nodes)
{     
    //ROS_WARN("Prepare to vis rrt result");
    visualization_msgs::Marker line_list, root;

    root.header.frame_id    = line_list.header.frame_id    = "/map";
    root.header.stamp       = line_list.header.stamp       = ros::Time::now();
    root.ns                 = "/root";
    line_list.ns            = "/edges";
    root.action             = line_list.action             = visualization_msgs::Marker::ADD;
    root.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    root.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    root.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    root.pose.orientation.z = line_list.pose.orientation.z = 0.0;
    root.id                 = line_list.id = 0;
    root.type               = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    
    root.scale.x      = root.scale.y      = root.scale.z = 0.25;
    line_list.scale.x = line_list.scale.y = line_list.scale.z = 0.025;

    root.color.a = 1.0;
    root.color.r = 0.0;
    root.color.g = 1.0;
    root.color.b = 1.0;

    line_list.color.a = 0.7;
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    line_list.points.clear();

    NodePtr nodeptr;
    for(int i = 0; i < int(nodes.size()); i++){
        nodeptr = nodes[i];
        if (nodeptr->preNode_ptr == NULL){ 
            geometry_msgs::Point r;
            r.x = nodeptr->coord(0);
            r.y = nodeptr->coord(1); 
            r.z = nodeptr->coord(2); 
            root.points.push_back(r);
            continue;
        }

        geometry_msgs::Point p_line;
        p_line.x = nodeptr->coord(0);
        p_line.y = nodeptr->coord(1); 
        p_line.z = nodeptr->coord(2); 
        line_list.points.push_back(p_line);
        
        p_line.x = nodeptr->preNode_ptr->coord(0);
        p_line.y = nodeptr->preNode_ptr->coord(1); 
        p_line.z = nodeptr->preNode_ptr->coord(2); 
        line_list.points.push_back(p_line);
    }

    _vis_rrt_star_pub.publish(root);
    _vis_rrt_star_pub.publish(line_list);
}

void visCommitTraj(MatrixXd polyCoeff)
{
    traj_commit_pts_pcd.points.clear();
    double t_s = max(0.0, (_odom.header.stamp - _start_time).toSec());      
    int idx;
    for (idx = 0; idx < _segment_num; ++idx){
      if (t_s > _Time(idx) && idx + 1 < _segment_num)
          t_s -= _Time(idx);
      else break;
    }

    double duration = 0.0;
    double t_ss;
    for(int i = idx; i < _segment_num; i++ )
    {
        t_ss = (i == idx) ? t_s : 0.0;
        for(double t = t_ss; t < _Time(i); t += 0.02){
            double t_d = duration + t - t_ss;
            if( t_d > _time_commit ) break;
            Vector3d traj_pt;
            getPosFromBezier( polyCoeff, t/_Time(i), i, traj_pt );
            
            PointXYZ pt_point;
            pt_point.x = _Time(i) * traj_pt(0); 
            pt_point.y = _Time(i) * traj_pt(1);
            pt_point.z = _Time(i) * traj_pt(2);
            
            traj_commit_pts_pcd.points.push_back(pt_point);
      }

      duration += _Time(i) - t_ss;
    }
    
    traj_commit_pts_pcd.width = traj_commit_pts_pcd.points.size();
    traj_commit_pts_pcd.height = 1;
    traj_commit_pts_pcd.is_dense = true;
    pcl::toROSMsg(traj_commit_pts_pcd, traj_commit_pts);
    traj_commit_pts.header.frame_id = "map";
    _vis_commit_traj_points.publish(traj_commit_pts);
}

void visBezierTrajectory(MatrixXd polyCoeff)
{   
    double traj_len = 0.0;
    int count = 0;

    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_pts_pcd.points.clear();
    for(int i = 0; i < _segment_num; i++ )
    {
        for (double t = 0.0; t <= _Time(i); t += 0.02, count += 1)
        {   
            Vector3d traj_pt;
            getPosFromBezier( polyCoeff, t / _Time(i), i, traj_pt );
            
            PointXYZ point;
            cur(0) = point.x = _Time(i) * traj_pt(0);
            cur(1) = point.y = _Time(i) * traj_pt(1);
            cur(2) = point.z = _Time(i) * traj_pt(2);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;

            traj_pts_pcd.points.push_back(point); 
        }
    }

    traj_pts_pcd.width = traj_pts_pcd.points.size();
    traj_pts_pcd.height = 1;
    traj_pts_pcd.is_dense = true;

    pcl::toROSMsg(traj_pts_pcd, traj_pts);
    traj_pts.header.frame_id = "map";
    
    _vis_traj_points.publish(traj_pts);

    ROS_INFO("[Demo] The length of the trajectory; %.3lfm.", traj_len);
}

void visFlightCorridor(MatrixXd path, VectorXd radius)
{           
    for (auto & mk: path_vis.markers) 
      mk.action = visualization_msgs::Marker::DELETE;

    _vis_corridor_pub.publish(path_vis);
    path_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "pcd_RRT/flight_corridor";
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 0.4;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    for(int i = 0; i < int(path.rows()); i++){
        mk.id = i;
        mk.pose.position.x = path(i, 0); 
        mk.pose.position.y = path(i, 1); 
        mk.pose.position.z = path(i, 2); 
        mk.scale.x = 2 * radius(i);
        mk.scale.y = 2 * radius(i);
        mk.scale.z = 2 * radius(i);
        
        path_vis.markers.push_back(mk);
    }

    _vis_corridor_pub.publish(path_vis);
}

void visCtrlPoint(MatrixXd polyCoeff)
{
    for (auto & mk: ctrlPt_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;

    _vis_ctrl_pts_pub.publish(ctrlPt_vis);

    ctrlPt_vis.markers.clear();
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "pcd_RRT/control_points";
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    int idx = 0;
    for(int i = 0; i < _segment_num; i++)
    {   
        int order = _poly_orderList[i];
        int ctrl_num = order + 1;
        
        for(int j = 0; j < ctrl_num; j++)
        {
            mk.id = idx;
            mk.pose.position.x = _Time(i) * polyCoeff(i, j);
            mk.pose.position.y = _Time(i) * polyCoeff(i, ctrl_num + j);
            mk.pose.position.z = _Time(i) * polyCoeff(i, 2 * ctrl_num + j);
            mk.scale.x = 0.25;
            mk.scale.y = 0.25;
            mk.scale.z = 0.25;
            ctrlPt_vis.markers.push_back(mk);
            idx ++;
        }
  }

  _vis_ctrl_pts_pub.publish(ctrlPt_vis);
}