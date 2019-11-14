#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <std_msgs/Bool.h>
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

ros::Publisher _all_map_pub, _finish_map_pub;
ros::Subscriber _map_sub, _auto_sub;

double  _resolution, _sense_rate;
double min_z = 100000;

bool _has_map  = false;
bool _end_map = false;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;   
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void rcvAutoModeCallBack(const std_msgs::Bool auto_mode);
void growPilar(pcl::PointXYZ searchPoint, double w);
bool _start_auto = false;
bool _finish_map = false;;
int cnt = 0;
pcl::PointCloud<pcl::PointXYZ> cloud, cloud_iter;
ros::Time _map_time;
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map){ 
   pcl::fromROSMsg(pointcloud_map, cloud);
   _map_time = pointcloud_map.header.stamp;
   _has_map = true;
}
void rcvAutoModeCallBack(const std_msgs::Bool auto_mode)
{
    _start_auto = auto_mode.data;
    if(_start_auto)
      ROS_INFO("[ActualMap] Start map optimizition.");
}
void pubSensedPoints()
{     
   //if(!_has_map || !_has_odom)
   if( !_has_map ) return;
   sensor_msgs::PointCloud2 map2pub;
   /* 
   if(flag2 < flag){
      flag2 = flag;
      pcl::toROSMsg(cloud, map2pub);
   }
   else if(flag2 == flag){
      
      for(auto pt_temp : cloud.points){
         min_z = fmin(min_z, pt_temp.z);
         if(pt_temp.z < 0) cnt++;
      }
      cout<<"minz: "<<min_z<<endl;
      // double move = max_z -1.2;
      // for(auto pt_temp : cloud.points){
      //   pt_temp.z = pt_temp.z - move -0.2;
      //    cloud_iter.points.push_back(pt_temp);
         
      // }
      if(3*cnt > cloud.points.size()){
         ROS_INFO("Under Path.");
         for(auto pt_temp : cloud.points){
            if(pt_temp.z >= -0.1){
               //pt_temp.z += 0.1;
               cloud_iter.points.push_back(pt_temp);
            }
         }
      }
      else{
         ROS_INFO("Above Path.");
         for(auto pt_temp : cloud.points){
            if(pt_temp.z >= min_z + 0.4){
               pt_temp.z = pt_temp.z - min_z - 0.6;
               cloud_iter.points.push_back(pt_temp);
            }
         }
      }
      
      pcl::toROSMsg(cloud_iter, map2pub);
      flag2++;
      ROS_WARN("Map Finished!");
   }
   else{
      pcl::toROSMsg(cloud_iter, map2pub);
      // map2pub.header.frame_id = "/map";
      // _all_map_pub.publish(map2pub);
   }*/
   if(!_finish_map){
      if(! _start_auto){
         pcl::toROSMsg(cloud, map2pub);
      }
      else{
         for(auto pt_temp : cloud.points){
            min_z = fmin(min_z, pt_temp.z);
            if(pt_temp.z < 0) cnt++;
         }
         cout<<"minz: "<<min_z<<endl;
         // double move = max_z -1.2;
         // for(auto pt_temp : cloud.points){
         //   pt_temp.z = pt_temp.z - move -0.2;
         //    cloud_iter.points.push_back(pt_temp);
            
         // }
         if(3*cnt > cloud.points.size()){
            ROS_INFO("Under Path.");
            for(auto pt_temp : cloud.points){
               if(pt_temp.z >= -0.1){
                  //pt_temp.z += 0.1;
                  cloud_iter.points.push_back(pt_temp);
               }
            }
         }
         else{
            ROS_INFO("Above Path.");
            for(auto pt_temp : cloud.points){
               if(pt_temp.z >= min_z + 0.4){
                  pt_temp.z = pt_temp.z - min_z - 0.6;
                  cloud_iter.points.push_back(pt_temp);
               }
            }
         }
         
         pcl::toROSMsg(cloud_iter, map2pub);
         ROS_WARN("[ActualMap] Map Finished!");
         _finish_map = true;
         std_msgs::Bool map_msg;
         map_msg.data = true;
         _finish_map_pub.publish(map_msg);
      }
   }    
   else{
      pcl::toROSMsg(cloud_iter, map2pub);
   }
   map2pub.header.frame_id = "/map";
   map2pub.header.stamp = _map_time;
   _all_map_pub.publish(map2pub);
}

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "manual_simple_scene");
   ros::NodeHandle n( "~" );

   _all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);                      
   _map_sub      = n.subscribe( "raw_map", 50, rcvPointCloudCallBack );
   _auto_sub  = n.subscribe( "auto",  1, rcvAutoModeCallBack );

   _finish_map_pub = n.advertise<std_msgs::Bool>("map_fin", 10);
   //n.param("map/obs_num",    _obs_num,  30);
   n.param("map/resolution", _resolution, 0.2);
   n.param("sensing/rate", _sense_rate, 1.0);

   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
      
      ros::spinOnce();
      pubSensedPoints();
      loop_rate.sleep();
   }
}