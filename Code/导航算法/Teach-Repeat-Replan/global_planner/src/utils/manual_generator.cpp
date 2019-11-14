#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
#include <math.h>
#include <random>

using namespace std;
using namespace Eigen;

ros::Publisher _all_map_pub;
ros::Subscriber _odom_sub;

int _obs_num;
double _x_size, _y_size, _z_size, _resolution, _sense_rate;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;

bool _has_map  = false;
bool _has_odom = false;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;   

void growPilar(pcl::PointXYZ searchPoint, double w);

void RandomMapGenerate()
{  
   double x, y; 
   //x    = (_x_l + _x_h) * 0.5;
   x = _x_l + 5;
   y = (_y_l + _y_h) * 0.5 - 1;

   while(y < _y_h+ 0.5)
   {
      y += _resolution;
      pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
      growPilar(searchPoint,0.08);
   }
   while(x < _x_h - 2)
   {
      x += _resolution*0.5;
      pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
      growPilar(searchPoint,0.08);
   }
    while(y > (_y_l + _y_h) * 0.5 -1)
   {
      y -= _resolution;
      pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
      growPilar(searchPoint,0.08);
   }
   while(x > _x_l + 5)
   {
      x -= _resolution;
      pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
      growPilar(searchPoint,0.08);
   }

   for(x = _x_l+7; x < _x_h-4; x+=_resolution){
      for(y = (_y_l + _y_h) * 0.5+1.5; y<_y_h-1.5; y+=_resolution){
         pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
         growPilar(searchPoint,0.15);
      }
   }
   

   cloudMap.width = cloudMap.points.size();
   cloudMap.height = 1;
   cloudMap.is_dense = true;

   _has_map = true;
   
   pcl::toROSMsg(cloudMap, globalMap_pcd);
   globalMap_pcd.header.frame_id = "map";
}

void growPilar(pcl::PointXYZ searchPoint, double w){
   //ROS_WARN("One Pilar Generated!");
   random_device rd;
   default_random_engine eng(rd());
   //uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
   uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

   pcl::PointXYZ pt_random;
   double h;
   //w    = rand_w(eng);

   searchPoint.x = floor(searchPoint.x/_resolution) * _resolution + _resolution / 2.0;
   searchPoint.y = floor(searchPoint.y/_resolution) * _resolution + _resolution / 2.0;

   int widNum = ceil(w/_resolution);
   for(int r = -widNum/2.0; r < widNum/2.0; r ++ )
   {
      for(int s = -widNum/2.0; s < widNum/2.0; s ++ )
      { 
         h = rand_h(eng); 
         int heiNum = 2.0 * ceil(h/_resolution);
         for(int t = 0; t < heiNum; t ++ ){
            pt_random.x = searchPoint.x + (r+0.0) * _resolution + 0.001;
            pt_random.y = searchPoint.y + (s+0.0) * _resolution + 0.001;
            pt_random.z =     (t+0.0) * _resolution * 0.5 + 0.001;
            cloudMap.points.push_back( pt_random );
         }
      }
   }
   
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
   _has_odom = true;
}

void pubSensedPoints()
{     
   //if(!_has_map || !_has_odom)
   if( !_has_map ) return;

   _all_map_pub.publish(globalMap_pcd);
}

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "manual_simple_scene");
   ros::NodeHandle n( "~" );

   _all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);                      
   _odom_sub      = n.subscribe( "odometry", 50, rcvOdometryCallbck );

   // n.param("init_state_x", _init_x,       0.0);
   // n.param("init_state_y", _init_y,       0.0);

   n.param("map/x_size",  _x_size, 50.0);
   n.param("map/y_size",  _y_size, 50.0);
   n.param("map/z_size",  _z_size, 5.0 );

   //n.param("map/obs_num",    _obs_num,  30);
   n.param("map/resolution", _resolution, 0.2);

   n.param("ObstacleShape/lower_rad", _w_l,   0.3);
   n.param("ObstacleShape/upper_rad", _w_h,   0.8);
   n.param("ObstacleShape/lower_hei", _h_l,   3.0);
   n.param("ObstacleShape/upper_hei", _h_h,   7.0);

   n.param("sensing/rate", _sense_rate, 1.0);

   _x_l = - _x_size / 2.0;
   _x_h = + _x_size / 2.0;

   _y_l = - _y_size / 2.0;
   _y_h = + _y_size / 2.0;

   RandomMapGenerate();
   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
      pubSensedPoints();
      ros::spinOnce();
      loop_rate.sleep();
   }
}