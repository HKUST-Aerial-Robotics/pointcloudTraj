#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <math.h>
#include <time.h>
#include <random>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PositionCommand.h>

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

ros::Publisher _all_map_pub, _local_map_pub;
ros::Subscriber _map_sub, _odom_sub, _cmd_sub;

vector<double> _state;
pcl::PointXYZ LstMapCenter;

int _obsNum;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _resolution, _sense_rate, _sensing_range, _field_of_view_vertical;
double _init_x, _init_y, _end_x, _end_y;

bool _is_map_ok   = false;
bool _is_has_odom = false;

sensor_msgs::PointCloud2 globalMap_pcd, localMap_pcd, localMap_nofloor_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

void RandomMapGeneration()
{    
   pcl::PointXYZ pt_random;

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
               
               if(sqrt( pow(pt_random.x - _init_x, 2) + pow(pt_random.y - _init_y, 2) ) < 2.0 
               || sqrt( pow(pt_random.x - _end_x,  2) + pow(pt_random.y - _end_y,  2) ) < 2.0 ) 
                  continue;
               
               cloudMap.points.push_back( pt_random );
            }
         }
   }

   cloudMap.width = cloudMap.points.size();
   cloudMap.height = 1;
   cloudMap.is_dense = true;

   ROS_WARN("[Map Generator] Finished generate random map ");
   //cout<<cloudMap.size()<<endl;
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
   pcl::toROSMsg(cloudMap, globalMap_pcd);
   globalMap_pcd.header.frame_id = "map";

   if(global_map_vis_cnt < 20 ){
      _all_map_pub.publish(globalMap_pcd);
      sleep(0.01);
   }

   global_map_vis_cnt ++;
   if(!_is_map_ok || !_is_has_odom)
      return;

   pcl::PointCloud<pcl::PointXYZ>::Ptr localMap(new pcl::PointCloud<pcl::PointXYZ>());
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

         if( sqrt(pow(ptInNoflation.x - LstMapCenter.x, 2) + pow(ptInNoflation.y - LstMapCenter.y, 2) + pow(ptInNoflation.z - LstMapCenter.z, 2) ) < _sensing_range )
            continue;
      }

   }

   localMap->width = localMap->points.size();
   localMap->height = 1;
   localMap->is_dense = true;
      
   pcl::toROSMsg(*localMap, localMap_pcd);
   localMap_pcd.header.frame_id = "map";
   _local_map_pub.publish(localMap_pcd);

   LstMapCenter = pcl::PointXYZ(_state[0], _state[1], _state[2]);
}


int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "random map generator");
   ros::NodeHandle nodehandle( "~" );

   _local_map_pub =
         nodehandle.advertise<sensor_msgs::PointCloud2>("RandomMap", 1);                            
   
   _all_map_pub   =
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
   
   nodehandle.param("sensing_radius", _sensing_range,          10.0);
   nodehandle.param("ObstacleNum",    _obsNum,                 30  );
   nodehandle.param("Resolution",     _resolution,             0.2 );
   nodehandle.param("SensingRate",    _sense_rate,             10.0);
   nodehandle.param("fov_vertical",   _field_of_view_vertical, 30.0); 
   
   nodehandle.param("init_x", _init_x,  0.0); 
   nodehandle.param("init_y", _init_y,  0.0); 
   nodehandle.param("end_x",  _end_x,   0.0); 
   nodehandle.param("end_y",  _end_y,   0.0); 

   RandomMapGeneration();
   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
   }
}