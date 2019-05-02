#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <math.h>

#include <time.h>
#include <sys/time.h>
#include <random>

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "quadrotor_msgs/PositionCommand.h"

#define inf 99999999.0

using namespace std;

pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
vector<int>     pointIdxRadiusSearch;
vector<float>   pointRadiusSquaredDistance;        

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double>  rand_x;
uniform_real_distribution<double>  rand_y;
uniform_real_distribution<double>  rand_w;
uniform_real_distribution<double>  rand_h;

ros::Publisher _all_map_pub, _local_map_pub, _local_map_nofloor_pub;
ros::Subscriber _map_sub, _odom_sub, _cmd_sub;

vector<double> _state;
pcl::PointXYZ LstMapCenter;

int _obsNum;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _resolution, _sense_rate, _sensing_range, _field_of_view_vertical;
double _init_x, _init_y;

bool _is_map_ok   = false;
bool _is_has_odom = false;
bool _has_ground;

sensor_msgs::PointCloud2 globalMap_pcd, localMap_pcd, localMap_nofloor_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap, cloudMap_nofloor;

void RandomMapGenerate()
{    
      pcl::PointXYZ pt_random;

      // #####################################################
      // Generate a random map, with vertical obstacles
      rand_x = uniform_real_distribution<double>(_x_l, _x_h );
      rand_y = uniform_real_distribution<double>(_y_l, _y_h );
      rand_w = uniform_real_distribution<double>(_w_l, _w_h);
      rand_h = uniform_real_distribution<double>(_h_l, _h_h);

      for(int i = 0; i < _obsNum; i ++){
         double x, y; 
         x    = rand_x(eng);
         y    = rand_y(eng);

         double w, h;
         w    = rand_w(eng);

         int widNum = ceil(w/_resolution);

         for(int r = -widNum / 2; r < widNum / 2; r ++ )
            for(int s = -widNum / 2; s < widNum / 2; s ++ ){
               h    = rand_h(eng);  
               //if(h < 1.0) continue;
               int heiNum = ceil(h/_resolution);
               for(int t = 0; t < heiNum; t ++ ){
                  pt_random.x = x + r * _resolution;
                  pt_random.y = y + s * _resolution;
                  pt_random.z = t * _resolution;
                  
                  if(sqrt( pow(pt_random.x - _init_x, 2) + pow(pt_random.y - _init_y, 2) ) < 2.0 ) 
                     continue;
                  
                  cloudMap.points.push_back( pt_random );
               }
            }
      }
      cloudMap_nofloor = cloudMap;
      cloudMap_nofloor.width = cloudMap_nofloor.points.size();
      cloudMap_nofloor.height = 1;
      cloudMap_nofloor.is_dense = true;

if(_has_ground)
{     
      ROS_WARN("[Map Generator] Has ground plane");
      uniform_real_distribution<double>  rand_g(0.0, 1.0);
      int x_all = (_x_h - _x_l) / _resolution;
      int y_all = (_y_h - _y_l) / _resolution;

      for(int i = -x_all; i < x_all; i ++)
      {
         for(int j = -y_all; j < y_all; j ++)
         {
            double p_g = rand_g(eng);
            
            if(p_g < 0.45) continue;
            pt_random.x = i * _resolution;
            pt_random.y = j * _resolution;
            pt_random.z = 0.0;
            cloudMap.points.push_back( pt_random );
         }
      }
}

      cloudMap.width = cloudMap.points.size();
      cloudMap.height = 1;
      cloudMap.is_dense = true;

      ROS_WARN("[Map Generator] Finished generate random map ");
      cout<<cloudMap.size()<<endl;
      kdtreeLocalMap.setInputCloud( cloudMap.makeShared() ); 

      _is_map_ok = true;
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
     if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return ;
     _is_has_odom = true;

     _state = {
         odom.pose.pose.position.x, 
         odom.pose.pose.position.y, 
         odom.pose.pose.position.z, 
         odom.twist.twist.linear.x,
         odom.twist.twist.linear.y,
         odom.twist.twist.linear.z,
         0.0, 0.0, 0.0
     };
}

int global_map_vis_cnt = 0;
void pubSensedPoints()
{     
      if(global_map_vis_cnt < 10 ){
         pcl::toROSMsg(cloudMap_nofloor, globalMap_pcd);
         globalMap_pcd.header.frame_id = "map";
         _all_map_pub.publish(globalMap_pcd);
      }

      global_map_vis_cnt ++;
      if(!_is_map_ok || !_is_has_odom)
         return;

      pcl::PointCloud<pcl::PointXYZ>::Ptr localMap(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr localMap_nofloor(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);

      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      
      pcl::PointXYZ ptInNoflation;

      if ( kdtreeLocalMap.radiusSearch (searchPoint, _sensing_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
         for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
            ptInNoflation = cloudMap.points[pointIdxRadiusSearch[i]];      
            if( abs(ptInNoflation.z - searchPoint.z ) >  _sensing_range * sin(_field_of_view_vertical/ 2.0 / 180.0 * M_PI) )
               continue;

            localMap->points.push_back(ptInNoflation);
            
            if(ptInNoflation.z > 0.05) // filter all points on the ground, just for visualization
               localMap_nofloor->points.push_back(ptInNoflation);

            if( sqrt(pow(ptInNoflation.x - LstMapCenter.x, 2) + pow(ptInNoflation.y - LstMapCenter.y, 2) + pow(ptInNoflation.z - LstMapCenter.z, 2) ) < _sensing_range )
               continue;
         }

      }
      else{
         ROS_ERROR("[Map Generator] No obstacles .");
         cout<<searchPoint.x<<" , "<<searchPoint.y<<" , "<<searchPoint.z<<endl;
         return;
      }

      localMap->width = localMap->points.size();
      localMap->height = 1;
      localMap->is_dense = true;
         
      pcl::toROSMsg(*localMap, localMap_pcd);
      localMap_pcd.header.frame_id = "map";
      _local_map_pub.publish(localMap_pcd);

      localMap_nofloor->width = localMap_nofloor->points.size();
      localMap_nofloor->height = 1;
      localMap_nofloor->is_dense = true;
         
      pcl::toROSMsg(*localMap_nofloor, localMap_nofloor_pcd);
      localMap_nofloor_pcd.header.frame_id = "map";
      _local_map_nofloor_pub.publish(localMap_nofloor_pcd);

      LstMapCenter = pcl::PointXYZ(_state[0], _state[1], _state[2]);
}


int main (int argc, char** argv) 
{        
      ros::init (argc, argv, "random map generator");
      ros::NodeHandle nodehandle( "~" );

      _local_map_pub =
            nodehandle.advertise<sensor_msgs::PointCloud2>("RandomMap", 1);                            
      
      _local_map_nofloor_pub =
            nodehandle.advertise<sensor_msgs::PointCloud2>("RandomMap_noFloor", 1);                            
      
      _all_map_pub =
            nodehandle.advertise<sensor_msgs::PointCloud2>("all_map", 1);  

      _odom_sub      = 
            nodehandle.subscribe( "odometry", 50, rcvOdometryCallbck );

      nodehandle.param("mapBoundary/lower_x", _x_l,       0.0);
      nodehandle.param("mapBoundary/upper_x", _x_h,     100.0);
      nodehandle.param("mapBoundary/lower_y", _y_l,       0.0);
      nodehandle.param("mapBoundary/upper_y", _y_h,     100.0);
      nodehandle.param("ObstacleShape/lower_rad", _w_l,   0.3);
      nodehandle.param("ObstacleShape/upper_rad", _w_h,   0.8);
      nodehandle.param("ObstacleShape/lower_hei", _h_l,   3.0);
      nodehandle.param("ObstacleShape/upper_hei", _h_h,   7.0);
      nodehandle.param("has_ground", _has_ground,        true);
      
      nodehandle.param("sensing_radius", _sensing_range,          10.0);
      nodehandle.param("ObstacleNum",    _obsNum,                 30  );
      nodehandle.param("Resolution",     _resolution,             0.2 );
      nodehandle.param("SensingRate",    _sense_rate,             10.0);
      nodehandle.param("fov_vertical",   _field_of_view_vertical, 30.0); 
      
      nodehandle.param("init_x",  _init_x,   0.0); 
      nodehandle.param("init_y",  _init_y,   0.0); 

      RandomMapGenerate();
      ros::Rate loop_rate(_sense_rate);
      while (ros::ok())
      {
           pubSensedPoints();
           ros::spinOnce();
           loop_rate.sleep();
      }
}