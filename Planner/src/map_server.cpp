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

pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
vector<int>     pointIdxRadiusSearch;
vector<float>   pointRadiusSquaredDistance;        

ros::Publisher _local_map_pub;
ros::Subscriber _odom_sub, _cmd_sub, _map_sub;

deque<nav_msgs::Odometry> _odom_queue;
vector<double> _state;
const size_t _odom_queue_size = 200;
nav_msgs::Odometry _odom;

double _sensing_rate, _sensing_range, _standard_deviation;

//ros::Timer vis_map;
bool map_ok = false;
bool _has_odom = false;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(map_ok) 
      return;
    
    pcl::PointCloud<pcl::PointXYZ> cloudIn;
    pcl::fromROSMsg(pointcloud_map, cloudIn);
    ROS_WARN("[Map Server] received the random map ");
    cout<<"map size: "<<cloudIn.size()<<endl;
    
    pcl::VoxelGrid<pcl::PointXYZ>  VoxelSampler;
    pcl::PointCloud<pcl::PointXYZ> Cloud_DS;
    
    VoxelSampler.setLeafSize(0.2f, 0.2f, 0.2f);
    VoxelSampler.setInputCloud( cloudIn.makeShared() );      
    VoxelSampler.filter( cloudMap );    

    //cloudMap = Cloud_DS;
    kdtreeLocalMap.setInputCloud( cloudMap.makeShared() ); 
    map_ok = true;

    cout<<"map size: "<<cloudMap.size()<<endl;
}

void rcvCmdCallbck(const quadrotor_msgs::PositionCommand cmd)
{   
    //cout<<"cmd"<<endl;
    _has_odom = true;
    _odom.pose.pose.position.x = cmd.position.x;
    _odom.pose.pose.position.y = cmd.position.y;
    _odom.pose.pose.position.z = cmd.position.z;

    _odom.twist.twist.linear.x = cmd.velocity.x;
    _odom.twist.twist.linear.y = cmd.velocity.y;
    _odom.twist.twist.linear.z = cmd.velocity.z;
    _odom.header = cmd.header;

    _odom.pose.pose.orientation.x = 0.0;
    _odom.pose.pose.orientation.y = 0.0;
    _odom.pose.pose.orientation.z = 0.0;
    _odom.pose.pose.orientation.w = 1.0;

    _state = {
        _odom.pose.pose.position.x, 
        _odom.pose.pose.position.y, 
        _odom.pose.pose.position.z, 
        _odom.twist.twist.linear.x,
        _odom.twist.twist.linear.y,
        _odom.twist.twist.linear.z,
        0.0, 0.0, 0.0
    };

    _odom_queue.push_back(_odom);
    while (_odom_queue.size() > _odom_queue_size) _odom_queue.pop_front();
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
    _odom = odom;
    _has_odom = true;

    _state = {
        _odom.pose.pose.position.x, 
        _odom.pose.pose.position.y, 
        _odom.pose.pose.position.z, 
        _odom.twist.twist.linear.x,
        _odom.twist.twist.linear.y,
        _odom.twist.twist.linear.z,
        0.0, 0.0, 0.0
    };
    
    _odom_queue.push_back(odom);
    while (_odom_queue.size() > _odom_queue_size) _odom_queue.pop_front();       
}

void compute (const pcl::PointCloud<pcl::PointXYZ> & input, pcl::PointCloud<pcl::PointXYZ> & output,
         double standard_deviation)
{
    output.points.resize (input.points.size ());
    output.header = input.header;
    output.width  = input.width;
    output.height = input.height;

    boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
    boost::normal_distribution<> nd (0, standard_deviation);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

    for (size_t point_i = 0; point_i < input.points.size (); ++point_i)
    {
        output.points[point_i].x = input.points[point_i].x + static_cast<float> (var_nor ());
        output.points[point_i].y = input.points[point_i].y + static_cast<float> (var_nor ());
        output.points[point_i].z = input.points[point_i].z + static_cast<float> (var_nor ());
    }
}

pcl::PointCloud<pcl::PointXYZ> localMap, CloudDS;
pcl::PointCloud<pcl::PointXYZ> output;
pcl::VoxelGrid<pcl::PointXYZ>  VoxelSampler;
void pubSensedPoints()
{     
    if(!map_ok || !_has_odom)
       return;

    pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
    //ros::Time time_1 = ros::Time::now();
    pcl::PointXYZ pt;

    if ( kdtreeLocalMap.radiusSearch (searchPoint, _sensing_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
       for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
       {
          pt = cloudMap.points[pointIdxRadiusSearch[i]];      
          localMap.points.push_back(pt);
       }
    }
    else
    {
       ROS_ERROR("[Map Server] No obstacles .");
       cout<<searchPoint.x<<" , "<<searchPoint.y<<" , "<<searchPoint.z<<endl;
       return;
    }

    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    VoxelSampler.setLeafSize(0.3f, 0.3f, 0.3f); 
    VoxelSampler.setInputCloud( localMap.makeShared() );      
    VoxelSampler.filter( CloudDS );   

    localMap = CloudDS;

    //compute (localMap, output, _standard_deviation);
    //pcl::toROSMsg(output, localMap_pcd);

    pcl::toROSMsg(localMap, localMap_pcd);
    localMap_pcd.header.frame_id = "map";
    _local_map_pub.publish(localMap_pcd);
    
    //cout<<"localMap size: "<<localMap.points.size()<<endl;
    /*ros::Time time_2 = ros::Time::now();
    ROS_WARN("[map server] time in publish a local map is %f", (time_2 - time_1).toSec());*/
}


int main (int argc, char** argv) {
        
      ros::init (argc, argv, "map_server");
      ros::NodeHandle n( "~" );

      _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("local_map", 1);                                                   

      _odom_sub = n.subscribe( "odometry",  50, rcvOdometryCallbck );
      _cmd_sub  = n.subscribe( "command",   50, rcvCmdCallbck );
      _map_sub  = n.subscribe( "PointCloud", 1, rcvPointCloudCallBack);

      n.param("LocalSensing/radius", _sensing_range, 20.0);
      n.param("LocalSensing/rate",   _sensing_rate, 10.0);
      n.param("LocalSensing/std",    _standard_deviation, 0.1);
      
      ros::Rate loop_rate(_sensing_rate);
      
      while (ros::ok())
      {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
      }

      return 0;
}