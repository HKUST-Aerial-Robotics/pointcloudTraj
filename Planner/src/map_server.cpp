#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <random>
//#include "boost.h"

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "pointcloudTraj/backward.hpp"

using namespace std;

namespace backward {
backward::SignalHandling sh;
}

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int>     _pointIdxRadiusSearch;
vector<float>   _pointRadiusSquaredDistance;        

ros::Publisher _local_map_pub;
ros::Subscriber _odom_sub, _cmd_sub, _map_sub;

double _sensing_rate, _sensing_range, _standard_deviation;
bool _use_accumulate_map;

double _pos[3];
bool _map_ok   = false;
bool _has_odom = false;

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map;
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & msg)
{   
    if( _map_ok ) 
        return;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_input;
    pcl::fromROSMsg(msg, cloud_input);
    
    pcl::VoxelGrid<pcl::PointXYZ>  VoxelSampler;
    VoxelSampler.setLeafSize(0.2f, 0.2f, 0.2f);
    VoxelSampler.setInputCloud( cloud_input.makeShared() );      
    VoxelSampler.filter( _cloud_all_map );    

    _kdtreeLocalMap.setInputCloud( _cloud_all_map.makeShared() ); 
    _map_ok = true;
}

void rcvCmdCallbck(const quadrotor_msgs::PositionCommand & msg)
{   
    _has_odom = true;
    _pos[0] = msg.position.x;
    _pos[1] = msg.position.y;
    _pos[2] = msg.position.z;
}

void rcvOdometryCallbck(const nav_msgs::Odometry & msg)
{
    _has_odom = true;
    _pos[0] = msg.pose.pose.position.x;
    _pos[1] = msg.pose.pose.position.y;
    _pos[2] = msg.pose.pose.position.z;
}

pcl::PointCloud<pcl::PointXYZ> _local_map, _local_map_ds;
pcl::VoxelGrid<pcl::PointXYZ>  _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;
void pubSensedPoints()
{     
    if(!_map_ok || !_has_odom)
       return;

    if( !_use_accumulate_map ){
        _local_map.points.clear();
    }

    pcl::PointXYZ searchPoint(_pos[0], _pos[1], _pos[2]);
    _pointIdxRadiusSearch.clear();
    _pointRadiusSquaredDistance.clear();
    //ros::Time time_1 = ros::Time::now();
    pcl::PointXYZ pt;

    if ( _kdtreeLocalMap.radiusSearch (searchPoint, _sensing_range, _pointIdxRadiusSearch, _pointRadiusSquaredDistance) > 0 )
    {
       for (size_t i = 0; i < _pointIdxRadiusSearch.size (); ++i)
       {
          pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];      
          _local_map.points.push_back(pt);
       }
    }
    else
    {
       ROS_ERROR("[Map Server] No obstacles .");
       cout<<searchPoint.x<<" , "<<searchPoint.y<<" , "<<searchPoint.z<<endl;
       return;
    }

    _local_map.width = _local_map.points.size();
    _local_map.height = 1;
    _local_map.is_dense = true;

    _voxel_sampler.setLeafSize(0.3f, 0.3f, 0.3f); 
    _voxel_sampler.setInputCloud( _local_map.makeShared() );      
    _voxel_sampler.filter( _local_map_ds );   

    _local_map = _local_map_ds;

    pcl::toROSMsg(_local_map, _local_map_pcd);
    _local_map_pcd.header.frame_id = "map";
    _local_map_pub.publish(_local_map_pcd);
}


int main (int argc, char** argv) {
        
      ros::init (argc, argv, "map_server");
      ros::NodeHandle n( "~" );

      _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("local_map", 1);                                                   

      _odom_sub = n.subscribe( "odometry",  50, rcvOdometryCallbck );
      _cmd_sub  = n.subscribe( "command",   50, rcvCmdCallbck );
      _map_sub  = n.subscribe( "pointCloud", 1, rcvPointCloudCallBack);

      n.param("LocalSensing/radius", _sensing_range, 20.0);
      n.param("LocalSensing/rate",   _sensing_rate, 10.0);
      n.param("LocalSensing/std",    _standard_deviation, 0.1);
      n.param("LocalSensing/use_accumulate_map", _use_accumulate_map, false);
      
      ros::Rate loop_rate(_sensing_rate);
      
      while (ros::ok())
      {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
      }

      return 0;
}