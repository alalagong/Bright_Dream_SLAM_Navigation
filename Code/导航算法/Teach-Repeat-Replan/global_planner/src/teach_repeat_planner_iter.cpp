#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/OptimalTimeAllocator.h>
#include <quadrotor_msgs/SpatialTemporalTrajectory.h>
#include <quadrotor_msgs/SimpleTrajectory.h>
#include <quadrotor_msgs/ReplanCheck.h>
#include <wheel_driver/WheelEncoderMsg.h>
#include <global_planner/utils/a_star.h>
#include <global_planner/utils/backward.hpp>
#include <global_planner/utils/poly_utils.h>
#include <global_planner/utils/bezier_base.h>
#include <global_planner/simple_optimizer.h>
#include <global_planner/utils/node_pid.h>
#include <global_planner/utils/myPoint.h>
#include <std_msgs/Bool.h>
using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}
#define PI_ 3.141592
#define F_KP 2.58  // P constant for PSD translation controller
#define F_KD 0.047  // D constant for PSD translation controller
#define F_KI 0.0  // S constant for PSD translation controller
#define R_KP 2.0  // P constant for PSD rotation controller
#define R_KD 0.1  // D constant for PSD rotation controller
#define R_KI 0.0  // S constant for PSD rotation controller
// simulation param from launch file
double _vis_traj_width;
double _x_size, _y_size, _z_size, _resolution, _inv_resolution;
double _cloud_margin, _minimize_order, _rounding_radius;
double _MAX_Vel, _MAX_Acc, _MAX_d_Acc, _wheel_r, _frequency, TOLERANCE, TOLERANCE_ANGLE;
int    _traj_order, _max_inf_iter, _max_clu_iter;
string _your_file_path;
double current_theta;
// useful global variables
bool _has_odom  = false;
bool _has_loop_odom  = false;
bool _has_map   = false;
bool _end_map   = false;
bool _has_traj  = false;
bool _set_target = false;
bool arrived = false;
bool _from_collision = false;
bool has_local_map(false);
bool inReplanning(false);
bool finish_cmd(false);
bool self_replan(false);
bool _start_auto(false);
//bool collision_status;
Vector3d _start_pt,  _end_pt, _first_pt;
Vector3d _map_lower, _map_upper;
bool cmd_send = false;
double _pt_max_x, _pt_min_x, _pt_max_y, _pt_min_y, _pt_max_z, _pt_min_z;
double _end_theta;
double _rho, _SAFE_DIST;
int _max_x_id, _max_y_id, _max_z_id;
int _traj_id = 1;
int _vis_iter_num = 0;
int msg_index = 0;
int freq_control = 0;
int idx_offset = 0;
vector<Vector3d> _manual_path, trrSegPts, visTurning,remainingTurning;
decomp_ros_msgs::PolyhedronArray _poly_array_msg;
vector<double> rList, longList;
vector<TurningNode> turningPts;
vector<double> Vel_Cmd, Ang_Cmd;
vector<Vector3d> planning_pt, coord_set;
// ros related
ros::Subscriber _map_sub, _odom_sub, _auto_sub, _joy_sub, _local_map_sub, _pid_sub, _loop_odom_sub;
ros::Publisher _vis_polytope_pub, _vis_traj_pub, _vis_grid_path_pub, _vis_inf_map_pub, _plan_vel_pub, _planning_coord_pub;
ros::Publisher _space_time_pub;

ros::Time _odom_time, _loop_odom_time, _traj_time_start, _traj_time_final, _time_update_odom, _time_update_loop_odom;
nav_msgs::Odometry _odom, _loop_odom;

// useful object
Bernstein      * _bezier_basis             = new Bernstein(); 
gridPathFinder * _path_finder              = new gridPathFinder();
//gridPathFinder * _path_finder_local              = new gridPathFinder();
polyhedronGenerator * _polyhedronGenerator = new polyhedronGenerator();

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void rcvOdometryCallBack(const nav_msgs::Odometry odom);
void rcvLoopOdometryCallBack(const nav_msgs::Odometry loop_odom);
void rcvAutoModeCallBack(const std_msgs::Bool auto_mode);
void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map );
void clearParam();
void trajPlanning();
void carPlanning();
void replanning();
void trajectoryGeneration(vector<Vector3d> turning);
bool isNext(Vector3d coord_1, Vector3d coord_2);
void turningTraj(double theta, double ccw, Vector3d start);
void straightTraj(double dist, Vector3d dir_norm, Vector3d start);
void stopTraj(double frequency, Vector3d startVel, Vector3d startCor);
void visCorridor( decomp_ros_msgs::PolyhedronArray & poly_array_msg );
void visGridPath();
void visPoint(vector<Vector3d> Points);
void visLineSegmentTrajectory(vector<Vector3d> Points);
void visLineSegment(vector<Vector3d> Points);
void clearVisualization(int grid_path_mk_num, int traj_mk_iter_num);
void commandHelper();
void findRange(Vector3d pt1, Vector3d pt2, vector<Vector3i> &localRange);
void trajCaculation(double dist, double frequency, vector<double> &vec_Vel, vector<double>  &vec_Dist);
vector<Vector3d> bezierGeneration(TurningNode turning);
bool PIDCaculation(double &angleCommand, double &speedCommand, double targetAngle, double targetDistance);
bool closeEnough(MyPoint* actual, double targetDistance, double targetAngle);
double calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum);
bool checkCollision();
geometry_msgs::TwistStamped sendVelocity(double linear, double angular);
geometry_msgs::Vector3Stamped sendCoordinate(Vector3d coord);
fstream path_record;

bool _WRITE_PATH, _READ_PATH, _INIT_PLAN;

class Coord{
    public:
    Coord(int x_, int y_, int z_){
        x = x_;
        y = y_;
        z = z_;
    }
    private:
    int x, y, z;
    friend bool operator < (const Coord& c1, const Coord& c2);
};
bool operator < (const Coord& c1, const Coord& c2){
    //return (c1.x == c2.x) && (c1.y == c2.y) && (c1.z == c2.z);
    if(c1.x < c2.x) return true;
    if(c1.x > c2.x) return false;
    if(c1.y < c2.y) return true;
    if(c1.y > c2.y) return false;
    if(c1.z < c2.z) return true;
    return false;
}
set<Coord> mySet;

Vector3i _pose_idx, _pose_idx_lst;


void rcvJoyCallBack(const sensor_msgs::Joy joy)
{   
    if(joy.buttons[7] == 1.0)
    {
        ROS_WARN("[trr_global_planner] Set destination.");
        
        if(_manual_path.size() == 0) return;
        _set_target = true;
        //_end_map = true;
        _end_pt   = _manual_path.back();
        _end_pt(2) = 0;
        _end_theta = 2 * atan2(_odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w);
    }
    else if(joy.buttons[6] == 1.0)
    {
        ROS_WARN("[trr_global_planner] Enter in autonomous mode");
        // if(_manual_path.size() == 0) return;
        _start_auto = true;
        if(_manual_path.size() == 0) return;
        _start_pt = _manual_path.front();
        _first_pt<<_loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y, 0;
        Vector3d init_dist_vec = _start_pt - _first_pt;
        if(init_dist_vec.norm()>2*_MAX_Vel){
            ROS_WARN("[trr_global_planner] Too far away from start point.");
            return;
        }
        visTurning.push_back(_first_pt);
        current_theta = 2 * atan2(_loop_odom.pose.pose.orientation.z, _loop_odom.pose.pose.orientation.w);

        carPlanning(); 
        // _start_pt = _manual_path.front();
        // Vector3d current_pt(_loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y, 0);
        // Vector3d init_dist_vec = _start_pt - current_pt;
        // if(init_dist_vec.norm()>2*_MAX_Vel){
        //     ROS_WARN("[trr_global_planner] Too far away from start point.");
        //     return;
        // }
        // visTurning.push_back(current_pt);
        // current_theta = 2 * atan2(_loop_odom.pose.pose.orientation.z, _loop_odom.pose.pose.orientation.w);

        //carPlanning(); 
        // ROS_WARN("[trr_global_planner] Enter in maunal flight mode");
        // clearVisualization(_manual_path.size(), _vis_iter_num);

        // _manual_path.clear();
        // _polyhedronGenerator->reset();
        // _has_traj = false;
        // _vis_iter_num = 0;
    }
}
void rcvAutoModeCallBack(const std_msgs::Bool auto_mode)
{
    //wheel_driver::WheelEncoderMsg _wheel_data = wheel_data;
    //int32_t  left = _wheel_data.left_encoding;
    //int32_t right = _wheel_data.right_encoding;
    //if(!_INIT_PLAN && left != 0 && right != 0)
    _end_map = auto_mode.data;   
    if(_end_map)
    {
        //_INIT_PLAN = true;
        ROS_WARN("[trr_global_planner] Map optimization finished.");
        //carPlanning(); 
    }
    
}

void rcvOdometryCallBack(const nav_msgs::Odometry odom)
{
    
    _odom = odom;
    _odom_time = odom.header.stamp;
    _time_update_odom = ros::Time::now();
    //ROS_INFO("Time gap between odom and now: %f/n", (_time_update_odom-_odom_time).toSec());
    _has_odom = true;

    Vector3d pos, pos_round;
    pos(0)  = odom.pose.pose.position.x;
    pos(1)  = odom.pose.pose.position.y;
    pos(2)  = odom.pose.pose.position.z;    

    _pose_idx = _path_finder->coord2gridIndex(pos);
    // if(has_local_map && !inReplanning){
    //     if(checkCollision()){
    //         ROS_WARN("Obstacle Detected!");
    //         replanning();
    //     }
    //     has_local_map = false;
    // }
    
    if(_set_target) return;
    if(_manual_path.size() == 0)
    {   
        pos_round = _path_finder->gridIndex2coord(_pose_idx);
        _manual_path.push_back(pos_round);
        _pose_idx_lst = _pose_idx;

        coord_set.push_back(pos_round);
    }
    else if( _pose_idx != _pose_idx_lst ) 
    {   
        if( _path_finder->IndexQuery(_pose_idx) > 0 ) 
            return;
        
        pos_round = _path_finder->gridIndex2coord(_pose_idx);
        
        if(isNext(_manual_path.back(), pos_round) == false)
        {
            _path_finder->AstarSearch(_manual_path.back(), pos_round);
            
            vector<Vector3d> localPath = _path_finder->getPath();

            for(auto coord: localPath)
            {   
                coord_set.   push_back(coord);
                _manual_path.push_back(coord);
            }

            _path_finder->resetMap();
        }
        else
        {
            coord_set.   push_back(pos_round);
            _manual_path.push_back(pos_round);
        }    

        _pose_idx_lst = _pose_idx;
    }
    else
        return;

    if( _has_traj || _READ_PATH) return;

    //if( _polyhedronGenerator->corridorIncreGeneration(coord_set, _poly_array_msg) == 1 )
    //    visCorridor(_poly_array_msg);
}
void rcvLoopOdometryCallBack(const nav_msgs::Odometry odom)
{
    _loop_odom = odom;
    _loop_odom_time = odom.header.stamp;
    _time_update_loop_odom = ros::Time::now();
    //ROS_INFO("Time gap between odom and now: %f/n", (_time_update_odom-_odom_time).toSec());
    _has_loop_odom = true;
   
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(! _end_map) return;
    if( _has_map ) return;
    ROS_WARN("[GlobalMapCallBack] Load global map.");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_inf;
    pcl::fromROSMsg(pointcloud_map, cloud);
    sensor_msgs::PointCloud2 map_inflation;
    //cout<<cloud.points.size()<<endl;
    if( (int)cloud.points.size() == 0)
        return;

    pcl::PointXYZ pt, pt_inf;

    int inf_step   = round(_cloud_margin * _inv_resolution);
    //int inf_step_z = max(1, inf_step / 2);
    int inf_step_z = 0;
    //cout<<"size:"<<(int)cloud.points.size()<<endl;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        //cout<<"idx:"<<idx<<endl;
        pt = cloud.points[idx];        
        for(int x = -inf_step ; x <= inf_step; x ++ )
        {
            for(int y = -inf_step ; y <= inf_step; y ++ )
            {
                for(int z = -inf_step_z; z <= inf_step_z; z ++ )
                {
                    double inf_x = pt.x + x * _resolution;
                    double inf_y = pt.y + y * _resolution;
                    double inf_z = pt.z + z * _resolution;
                    Vector3d vec_inf(inf_x, inf_y, inf_z);
                    Vector3i idx_inf = _path_finder->coord2gridIndex(vec_inf);

                    // set in obstacle points
                    _path_finder->setObs(inf_x, inf_y, inf_z);
                    _polyhedronGenerator->setObs(idx_inf);

                    // rounding for visualizing the grid map
                    Vector3d coor_round = _path_finder->gridIndex2coord( idx_inf );
                    pt_inf.x = coor_round(0);
                    pt_inf.y = coor_round(1);
                    pt_inf.z = coor_round(2);
                    cloud_inf.points.push_back(pt_inf);
                }
            }
        }
    }

    _polyhedronGenerator->finishMap();

    cloud_inf.width    = cloud_inf.points.size();
    cloud_inf.height   = 1;
    cloud_inf.is_dense = true;

    pcl::toROSMsg(cloud_inf, map_inflation);
    map_inflation.header.frame_id = "/map";
    _vis_inf_map_pub.publish(map_inflation);

    _has_map = true;        
    
    if( _polyhedronGenerator->corridorIncreGeneration(coord_set, _poly_array_msg) == 1 )
        visCorridor(_poly_array_msg); 

    //carPlanning();
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
    if(!_start_auto) return;
    //freq_control++;
    //if(freq_control % 20 != 0) return;

    mySet.clear();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_inf;
    pcl::fromROSMsg(pointcloud_map, cloud);
    sensor_msgs::PointCloud2 map_inflation;

    if( (int)cloud.points.size() == 0)
        return;

    pcl::PointXYZ pt, pt_inf;

    int inf_step   = round(_cloud_margin * _inv_resolution);
    //int inf_step_z = max(1, inf_step / 2);
    int inf_step_z = 0;
    //cout<<"local cloud size:"<<(int)cloud.points.size()<<endl;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        
        for(int x = -inf_step ; x <= inf_step; x ++ )
        {
            for(int y = -inf_step ; y <= inf_step; y ++ )
            {
                for(int z = -inf_step_z; z <= inf_step_z; z ++ )
                {
                    double inf_x = pt.x + x * _resolution;
                    double inf_y = pt.y + y * _resolution;
                    double inf_z = pt.z + z * _resolution;

                    Vector3d vec_inf(inf_x, inf_y, inf_z);
                    Vector3i idx_inf = _path_finder->coord2gridIndex(vec_inf);
                    Coord cur(idx_inf(0), idx_inf(1), idx_inf(2));
                    mySet.insert(cur);

                    Coord test(idx_inf(0), idx_inf(1), idx_inf(2));
                    //cout<<idx_inf<<endl;
                    // //rounding for visualizing the grid map
                    Vector3d coor_round = _path_finder->gridIndex2coord( idx_inf );
                    // pt_inf.x = coor_round(0);
                    // pt_inf.y = coor_round(1);
                    // pt_inf.z = coor_round(2);
                    // cloud_inf.points.push_back(pt_inf);
                }
            }
        }
    }

    cloud_inf.width    = cloud_inf.points.size();
    cloud_inf.height   = 1;
    cloud_inf.is_dense = true;
    //ROS_INFO("Local map loaded. Set size: %i.", mySet.size());
    has_local_map = true;
    
}

bool isNext(Vector3d coord_1, Vector3d coord_2)
{
    Vector3i index_1 = _path_finder->coord2gridIndex(coord_1);
    Vector3i index_2 = _path_finder->coord2gridIndex(coord_2);

    if( abs(index_1(0) - index_2(0)) <= 1 
     && abs(index_1(1) - index_2(1)) <= 1 
     && abs(index_1(2) - index_2(2)) <= 1 )
        return true;

    return false;
}
void carPlanning()
{   
    if( _has_map == false || _has_odom == false) 
        return;
    //double current_dir = 2 * atan2(_odom.pose.pose.orientation.w, _odom.pose.pose.orientation.z);
    decomp_cvx_space::FlightCorridor corridor = _polyhedronGenerator->getCorridor();
    vector<decomp_cvx_space::Polytope> polyhedrons  = corridor.polyhedrons;
    vector<double>   time = corridor.durations;
    int segment_num  = polyhedrons.size();


    _start_pt = _manual_path.front();
    //_end_pt   = _manual_path.back();

    _start_pt(2) = 0;
    _end_pt(2) = 0;

    trrSegPts.push_back(_start_pt);
    trrSegPts.push_back(_end_pt);
    
    simpleTrajOptimizer  * line_optimizer = new simpleTrajOptimizer();
    line_optimizer->lineSegmentGeneration(corridor, trrSegPts, turningPts,  _MAX_Vel, _MAX_Acc);

    visTurning.push_back(_start_pt);
    for(auto turn : turningPts){
        //visTurning.push_back(turn.preCoord);
        //visTurning.push_back(turn.nextCoord);
        visTurning.push_back(turn.centerCoord);
        
    }
    visTurning.push_back(_end_pt);
    cout<<visTurning.front()<<endl;
    ROS_WARN("Polyhedron number: %i, Intersection number: %i. Turning points number: %i", segment_num, turningPts.size(), visTurning.size());
    //visPoint(visTurning);
    //visLineSegment(visTurning);
    visLineSegmentTrajectory(visTurning);
    
    msg_index = 0;
    Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
    trajectoryGeneration(visTurning);
    ROS_WARN("Command Velocity Numbers: %i, Angular Numbers: %i, Coord Numbers: %i", Vel_Cmd.size(), Ang_Cmd.size(), planning_pt.size());

    delete line_optimizer;
    _has_traj = true;
}
int iter = 0;
// void replanning(){
//     msg_index = 0;
//     Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();

//     Vector3d startVel(_odom.twist.twist.linear.x, _odom.twist.twist.linear.y, 0);
//     Vector3d startCor(_odom.pose.pose.position.x, _odom.pose.pose.position.y, 0);
//     double startBeta = atan2(_odom.twist.twist.linear.y,_odom.twist.twist.linear.x );
//     double v_value = startVel.norm();
//     //Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
//     double PI = 3.1415;
//     double frequency = _frequency;
//     if(!finish_cmd)
//         stopTraj(frequency, startVel, startCor);
//     Vector3d cur_turn;
//     double cur_beta = startBeta;
//     Vector3d vec_beta;
//     int count = 0;
    
//     while(collision_status){
//         cur_turn = planning_pt.back();
//         turningTraj(PI/2, 1, cur_turn);
//         cur_beta -= PI/2;
//         vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
//         straightTraj(0.2, vec_beta, cur_turn);
//         cur_turn = planning_pt.back();
//         turningTraj(PI/2, -1, cur_turn);
//         cur_beta += PI/2;
//         vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
//         count++;
//         collision_status = checkCollision(); 
//     }
//     straightTraj(0.2, vec_beta, cur_turn);
//     cur_turn = planning_pt.back();
//     turningTraj(PI/2, -1, cur_turn);
//     cur_beta += PI/2;
//     vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
//     collision_status = checkCollision();
//     while(collision_status){
//         turningTraj(PI/2, 1, cur_turn);
//         cur_beta -= PI/2;
//         vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
//         straightTraj(0.2, vec_beta, cur_turn);
//         cur_turn = planning_pt.back();
//         turningTraj(PI/2, -1, cur_turn);
//         cur_beta += PI/2;
//         vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
//         count++;
//         collision_status = checkCollision(); 
//     }
//     straightTraj(0.2*count, vec_beta, cur_turn);
//     Vector3d lst_pt = planning_pt.back();
//     remainingTurning.insert(remainingTurning.begin(), lst_pt);
//     trajectoryGeneration(remainingTurning);
//     inReplanning = false;
// }
void replanning(){
    if(!_start_auto) return;
    if(arrived) return;
    //if(!has_local_map) return;
    freq_control = (freq_control+1) % 50;
    if(freq_control != 0) return;
    //if(msg_index != 0 && Ang_Cmd[msg_index] != 0) return;
    if(msg_index != 0 && Ang_Cmd[msg_index] != 0) {
        //ROS_WARN("[Replanner] Turning... Return Replanning.");
        return;
    }
    bool collision_status = checkCollision();
    has_local_map = false;
    if(!inReplanning){
        if(!collision_status)
            return;
        else{
            msg_index = 0;
            Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
            inReplanning = true;
            ROS_WARN("[Replanner] Collision Detected! Stop Trajectory.");
            return;
        }
    }
    
    else{
        bool moving = (fabs(_odom.twist.twist.linear.x) > 0.03 || fabs(_odom.twist.twist.linear.y > 0.03));
        //ROS_INFO("[Replanner] VELOCITY: X: %f, Y: %f.", _odom.twist.twist.linear.x, _odom.twist.twist.linear.y);
        if(moving) {
            cout<<"[Replanner] Moving."<<endl;
            return;
        }
        //Vector3d startVel(_odom.twist.twist.linear.x, _odom.twist.twist.linear.y, 0);
        Vector3d startCor(_loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y, 0);
        //double startBeta = atan2(_odom.twist.twist.linear.y,_odom.twist.twist.linear.x );
        //double v_value = startVel.norm();
        double startBeta = 2 * atan2(_loop_odom.pose.pose.orientation.z, _loop_odom.pose.pose.orientation.w);
        double frequency = _frequency;
        Vector3d cur_turn;
        double cur_beta = startBeta;
        Vector3d vec_beta;
        cur_turn = startCor;
        vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
        collision_status = checkCollision();
        
        cout<<"[Replanner] collision status: "<<collision_status<<endl;
        if(!finish_cmd){
            cout<<"[Replanner] Commands not finished."<<endl;
            return;
        }
        else{
            cout<<"[Replanner] Previous Command Finish. New Command Generated."<<endl;
            msg_index = 0;
            Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
            if(collision_status){
                ROS_WARN("Turn 90 degree counter-clockwise.");
                _from_collision = true;
                turningTraj(PI_/2, 1, cur_turn);
                // int u = 0;
                // while(u<100){
                //     Vel_Cmd.push_back(0);
                //     Ang_Cmd.push_back(0);
                //     planning_pt.push_back(planning_pt.back());
                //     u++;
                // }
                cur_beta -= PI_/2;
                vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
                //iter = 0;
                ROS_INFO("[Replanner]{status} Command Size: %i.", Vel_Cmd.size());
                return; 
                //inReplanning = false;
            }
            else if(!collision_status){
                //inReplanning = false;
                if(_from_collision){
                    ROS_WARN("[Replanner] From Collision. Go Straight. Turn 90 degree clockwise.");
                    straightTraj(0.4, vec_beta, cur_turn);
                    cur_turn = planning_pt.back();
                    turningTraj(PI_/2, -1, cur_turn);
                    ROS_INFO("[Replanner]{not status} Commanf Size: %i.", Vel_Cmd.size());
                    _from_collision = false;
                }
                else{
                    iter++;
                    cout<<"Previous No Collision. iter: "<<iter<<endl;
                    if(iter == 1){
                        ROS_WARN("[Replanner] Go Straight. Turn 90 degree clockwise.");
                        straightTraj(0.4, vec_beta, cur_turn);
                        cur_turn = planning_pt.back();
                        turningTraj(PI_/2, -1, cur_turn);
                        ROS_INFO("[Replanner]{not status} Commanf Size: %i.", Vel_Cmd.size()); 
                        _from_collision = false;    
                    }
                    else if(iter == 2){
                        ROS_WARN("[Replanner] Go Straight. Turn 90 degree counter-clockwise.");
                        straightTraj(0.3, vec_beta, cur_turn);
                        cur_turn = planning_pt.back();
                        turningTraj(PI_/2, 1, cur_turn);
                        ROS_INFO("[Replanner]{not status} Commanf Size: %i.", Vel_Cmd.size());  
                         _from_collision = false;   
                    }
                    else if(iter >= 3){
                        ROS_WARN("[Replanner] Quit Replanning.");
                        msg_index = 0;
                        Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
                        //Vector3d lst_pt = planning_pt.back();
                        Vector3d lst_pt(_loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y, 0);
                        remainingTurning.insert(remainingTurning.begin(), lst_pt);
                        trajectoryGeneration(remainingTurning);
                        _from_collision = false;   
                        inReplanning = false;
                        iter = 0;
                        return;
                    }
                }   
            }
        }   
    }
}   
    // if(!finish_cmd && !collision_status) return;
   
    // inReplanning = true;

    // msg_index = 0;
    // Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();

    // //if(vel!=0) return;
 
    // Vector3d startVel(_odom.twist.twist.linear.x, _odom.twist.twist.linear.y, 0);
    // Vector3d startCor(_odom.pose.pose.position.x, _odom.pose.pose.position.y, 0);
    // double startBeta = atan2(_odom.twist.twist.linear.y,_odom.twist.twist.linear.x );
    // double v_value = startVel.norm();
    // double PI = 3.1415;
    // double frequency = _frequency;
    // Vector3d cur_turn;
    // double cur_beta = startBeta;
    // Vector3d vec_beta;
    // cur_turn = startCor;
    // vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;

    // if(!collision_status && _odom.twist.twist.linear.x < 0.1 && _odom.twist.twist.linear.y < 0.1){ 
    //     straightTraj(0.2, vec_beta, cur_turn);
    //     cur_turn = planning_pt.back();
    //     turningTraj(PI/2, -1, cur_turn);
    //     iter++;
    //     if(iter<3){
    //         return;
    //     }
    //     else{
    //         Vector3d lst_pt = planning_pt.back();
    //         remainingTurning.insert(remainingTurning.begin(), lst_pt);
    //         trajectoryGeneration(remainingTurning);
    //         inReplanning = false;
    //         return;
    //     }
    // }
    // if(collision_status && _odom.twist.twist.linear.x < 0.1 && _odom.twist.twist.linear.y < 0.1)) {
    //     turningTraj(PI/2, 1, cur_turn);
    //     cur_beta -= PI/2;
    //     vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;
    //     straightTraj(0.2, vec_beta, cur_turn);
    //     cur_turn = planning_pt.back();
    //     turningTraj(PI/2, -1, cur_turn);
    //     cur_beta += PI/2;
    //     vec_beta<<cos(cur_beta), sin(cur_beta), 0.0;    
    // }

    

void trajectoryGeneration(vector<Vector3d> turning){  
    //if(!replan)
    //    Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
    Vector3d distv, dir_norm;
    double vel, theta, theta_ori;
    double dist, ccw;
    Vector3d v1, v2;
    remainingTurning = turning;

    current_theta = 2 * atan2(_loop_odom.pose.pose.orientation.z, _loop_odom.pose.pose.orientation.w);
    //cout<<"current theta: "<<current_theta<<endl;
    for(int i = 0; i < turning.size(); i++){
        vel = 0;        
        //Turning Part
        
        if(i!=0){
            v1 = turning[i] - turning[i-1];   
        }else{
            v1 << cos(current_theta), sin(current_theta), 0;  
            //cout<<"Initial direction:"<<v1<<endl;
            //cout<<"First direction:" << v2<<endl;
        }
        if(i<turning.size()-1){
            v2 = turning[i+1] - turning[i];
        }
        else{
            v2 << cos(_end_theta), sin(_end_theta), 0;  
            //cout<<"Final Theta: "<<v2<<endl;
        }
        //cout<<"v1:"<<v1<<"v2:"<<v2<<endl; 
        if(v1 != v2){ 
            theta_ori = v1.adjoint()*v2;
            theta = acos(theta_ori/(v1.norm()*v2.norm()));
            ccw = v1(0)*v2(1) - v1(1)*v2(0);
            //ROS_INFO("%i th turning angle: %f", i, theta);
            turningTraj(theta, ccw, turning[i]);
        }

        //Line Part 
        if(i<turning.size()-1){
            distv = turning[i+1] - turning[i];
            dist = distv.norm();
            dir_norm = distv / dist;
            cout<<"Distance:"<<dist<<endl;
            straightTraj(dist, dir_norm, turning[i]);    
        }
        
    }

    visPoint(turning);
    
}
void stopTraj(double frequency, Vector3d startVel, Vector3d startCor){
    double _Dcc = 0.3;
    double v_value = startVel.norm();
    Vector3d v_norm = startVel/v_value;
    double t1 = v_value/_Dcc;
    double freq_inv = 1/frequency;
    double vel, d;
    Vector3d curCord;
    for(double t = 0; t <= t1; t += 1/frequency){
        vel = v_value - _Dcc*t;
        d = v_value*t - 0.5*_Dcc*t*t;
        curCord = startCor + d * v_norm;
        vel = vel > 0 ? vel : 0;
        Vel_Cmd.push_back(vel);
        Ang_Cmd.push_back(0);
        planning_pt.push_back(curCord);
    }
}
void turningTraj(double theta, double ccw, Vector3d start){
    vector<double> tempVel, tempDist;
    double frequency = _frequency;
    double dist = theta * _wheel_r;
    theta = (ccw > 0 ? 1 : -1)*theta;
    //cout<<"turning angle:"<<theta<<endl;
    trajCaculation(dist, frequency, tempVel, tempDist);
    int z = 0;
    while(z<50){
        tempVel.push_back(0);
        tempDist.push_back(tempDist.back());
        z++;
    }
    for(auto vel_temp : tempVel){
        Vel_Cmd.push_back(vel_temp);
        Ang_Cmd.push_back(theta);
        planning_pt.push_back(start);
    }
    tempVel.clear();
    tempDist.clear();
}
void straightTraj(double dist, Vector3d dir_norm, Vector3d start){
    vector<double> tempVel, tempDist;
    Vector3d pt;
    double frequency = _frequency;
    trajCaculation(dist, frequency, tempVel, tempDist);
    int u = 0;
    while(u<50){
        tempVel.push_back(0);
        tempDist.push_back(tempDist.back());
        u++;
    }

    for(auto vel_temp : tempVel){
        Vel_Cmd.push_back(vel_temp);
        Ang_Cmd.push_back(0);
    }
    for(auto d : tempDist){
        pt = start + d * dir_norm;
        planning_pt.push_back(pt);
    }
    tempVel.clear();
    tempDist.clear();
}
void trajCaculation(double dist, double frequency, vector<double> &vec_Vel, vector<double>  &vec_Dist){
    double freq_inv = 1/frequency;
    double v0, t0, t1, t2, s0, vel_cur;
    //double _Vel = _MAX_Vel;
    //double _Acc = _MAX_Acc;
    //double _Dcc = _MAX_d_Acc;
    //double _Rad = _MAX_Rad; 
    double _Vel = 0.3;
    double _Acc = 0.3;
    double _Dcc = 0.3;
    double vel = 0;
    double distunit = 0;
    //vector<double> vec_Vel, vec_Dist;
    s0 = 0.5 * _Vel * _Vel * (1/_Dcc + 1/_Acc);
    if(dist > s0){
        //cout<<"Can Achieve Set Max Speed."<<endl;
        t0 = _Vel/_Acc;
        t1 = (dist-s0)/_Vel + t0;
        t2 = t1 + _Vel/_Dcc;
        //cout<<"deccelerate time:"<<t1<<endl;;
        //cout<<"ending time:"<<t2<<endl;
        for(double t = 0; t <= t2; t += freq_inv){
            //cout<<t;
            if(t<t0){
                vel = _Acc*t;
                distunit = 0.5 * _Acc * t * t;
            }
            else if(t<t1){
                vel = _Vel;
                distunit = _Vel * t - 0.5 * _Vel * t0;
            }
            else{
                vel = _Vel - (t-t1)*_Dcc;
                distunit = _Vel * (t - 0.5 * t0) - 0.5 * _Dcc * pow(t-t1, 2);
            }
            //cout<<"vel:"<<vel<<endl;
            vel = vel > 0 ? vel : 0;
            vec_Vel.push_back(vel);
            vec_Dist.push_back(distunit);
        }
    }
    else{
        
        v0 = sqrt(_Acc*_Dcc*dist*2/(_Acc+_Dcc));
        t0 = sqrt(_Dcc*dist*2/((_Acc+_Dcc)*_Acc));
        t1 = sqrt(2*dist*(1/_Acc+1/_Dcc));
        //cout<<"Can't achieve! Max speed is:"<<v0<<endl;
        //cout<<"ending time:"<<t1<<endl;
        for(double t = 0; t <= t1; t += 1/frequency){
            if(t<t0){
                vel = _Acc*t;
                distunit = 0.5 * _Acc * t * t;
            } 
            else{
                vel = v0 - _Dcc*(t-t0);
                distunit = v0 * (t - 0.5 *t0) - 0.5 * _Dcc * pow(t-t0, 2); 
            } 
            //cout<<"vel:"<<vel<<endl;
            vel = vel > 0 ? vel : 0;
            vec_Vel.push_back(vel);
            vec_Dist.push_back(distunit);
        }
    }
}
bool checkCollision()
{
    int cnt = 0;
    double dist = _SAFE_DIST;
    Vector3d check_pt, check_lb, check_ru, pos;
    Vector3i check_idx, pos_idx, lb_idx, ru_idx;
    double _current_theta = 2 * atan2(_odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w);
    pos(0)  = _loop_odom.pose.pose.position.x;
    pos(1)  = _loop_odom.pose.pose.position.y;
    pos(2)  = _loop_odom.pose.pose.position.z;  
    //cout<<"Direction:"<<_current_theta<<endl;
    pos_idx = _path_finder->coord2gridIndex(pos);
    int planning_check_idx = msg_index-idx_offset+50;
    check_pt(0) = pos(0) + dist * cos(_current_theta);
    check_pt(1) = pos(1) + dist * sin(_current_theta);
    check_pt(2) = pos(2);
    // if(planning_check_idx < 0){
    //     return false;
    // } 
    // else if(planning_check_idx < planning_pt.size()){
    //     check_pt = planning_pt[planning_check_idx];
    // }    
    // else{
    //     check_pt = planning_pt.back();
    // }

    
    double theta_normal_ori = _current_theta + PI_/2;
    double theta_normal = theta_normal_ori > PI_ ? theta_normal_ori - PI_ : theta_normal_ori;

    Vector3d normal_vec;
    normal_vec << cos(theta_normal), sin(theta_normal), 0;
    //ROS_INFO("Normal direction: %f, %f", normal_vec(0), normal_vec(1));
    check_lb(0) = check_pt(0) - 0.4 * normal_vec(0);
    check_lb(1) = check_pt(1) - 0.4 * normal_vec(1);
    //check_lb(2) = check_pt(2) - 0.15;
    check_lb(2) = check_pt(2);
    check_ru(0) = check_pt(0) + 0.4 * normal_vec(0);
    check_ru(1) = check_pt(1) + 0.4 * normal_vec(1);
    //check_ru(2) = check_pt(2) + 0.2;
    check_ru(2) = check_pt(2);

    //vector<Vector3d> vis_vec;
    //vis_vec.push_back(check_lb);
    //vis_vec.push_back(check_ru);
    //visPoint(vis_vec);
    check_idx = _path_finder->coord2gridIndex(check_pt);
    lb_idx = _path_finder->coord2gridIndex(check_lb);
    ru_idx = _path_finder->coord2gridIndex(check_ru);
    //check_pt_round = _path_finder->gridIndex2coord(check_idx);
    //cout<<"check_pt_index:"<<check_idx<<endl;
    //ROS_INFO("check_pt: %f, %f, %f.", check_pt(0), check_pt(1), check_pt(2));
    //cout<<"pose index:"<<pos_idx<<endl;
   
    //_path_finder->AstarSearch(check_lb, check_ru);
    vector<Vector3i> localRange;  
    localRange.push_back(lb_idx);
    localRange.push_back(ru_idx);
    findRange(check_lb, check_ru, localRange);     
    //vector<Vector3i> localRange = _path_finder->getPathIndx();
    //cout<<"2D range size: "<<localRange.size()<<endl;

    set<Coord>::iterator it2;
    int minz = max(check_idx(2)-1, 0);
    for(int k = minz; k <= check_idx(2) + 2; k++){
        for(auto pt: localRange){
            Coord c0(pt(0),pt(1),k);
            it2 = mySet.find(c0);
            if(it2 != mySet.end())
                cnt++;
        }
    }
    
    //int total = (maxx-minx+1)*(maxy-miny+1)*(maxz-minz+1);
    int total = localRange.size() * (check_idx(2) -minz +3);
    //cout<<"total: "<< total<<"   cnt: "<<cnt<<endl;
    if(cnt>total/2)
        return true;
    else
        return false;    
}

void findRange(Vector3d pt1, Vector3d pt2, vector<Vector3i> &localRange){
    //vector<Vector3i> localRange;
    Vector3i pt1_i = _path_finder->coord2gridIndex(pt1);
    Vector3i pt2_i = _path_finder->coord2gridIndex(pt2);
    Vector3d mid_pt = (pt1 + pt2)/2;
    Vector3i mid_i = _path_finder->coord2gridIndex(mid_pt);
    if(mid_i != pt1_i && mid_i != pt2_i){
        localRange.push_back(mid_i);
        findRange(pt1, mid_pt, localRange);
        findRange(pt2, mid_pt, localRange);
    }else
    {
        return;
    }
}

void checkPosition(Vector3d targetPt){
    
    Vector3d current_pt(_loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y, 0);
    Vector3d dist_vec = targetPt - current_pt;
    double current_dist = dist_vec.norm();
    if(current_dist > 2.5 * _wheel_r){
        ROS_WARN("[checkPosition] Left Trajectory. Stop. Replanning...");
        //cout<<"current"
        msg_index = 0;
        Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
        bool moving = (fabs(_odom.twist.twist.linear.x) > 0.02 || fabs(_odom.twist.twist.linear.y > 0.02));
        ROS_INFO("[checkPosition] VELOCITY: X: %f, Y: %f.", _odom.twist.twist.linear.x, _odom.twist.twist.linear.y);
        if(!moving){
            Vector3d startCor(_loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y, 0);
            remainingTurning.insert(remainingTurning.begin(), startCor);
            trajectoryGeneration(remainingTurning);  
            ROS_WARN("[checkPosition] Finish Replanning. New turning size: %i", remainingTurning.size());
            // for(int i = 0; i<remainingTurning.size(); i++){
            //     Vector3d p = remainingTurning[i];
            //     ROS_INFO("The %i th turning coord is: %f, %f, %f.", i, p(0), p(1), p(2));
            // }
        }
        else{
            ROS_WARN("[checkPosition] Still Moving.");
            return;
        }

        // double frequency = _frequency;
        // stopTraj(frequency, startVel, startCor);
        //remainingTurning.insert(remainingTurning.begin(), planning_pt.back());
        //trajectoryGeneration(remainingTurning);    
    }
    //else
        //ROS_WARN("On Trajectory.");
}

vector<Vector3d> bezierGeneration(TurningNode turning){
   
   vector<Vector3d> res;
   Vector3d temp1, temp2, result;
   Vector3d p1 = turning.preCoord;
   Vector3d p2 = turning.centerCoord;
   Vector3d p3 = turning.nextCoord;

    Vector3d a = p1 - 2*p2 + p3;
    Vector3d b = 2*p2 - 2*p1;
    double A = 4*(a(0)*a(0) + a(1)*a(1));
    double B = 4*(a(0)*b(0) + a(1)*b(1));
    double C = b(0)*b(0) + b(1)*b(1);
    
    double Sabc = 2*sqrt(A+B+C);
    double A_2 = sqrt(A);
    double A_32 = 2*A*A_2;
    double C_2 = 2*sqrt(C);
    double BA = B/A_2;
 
    double length = (A_32*Sabc + 
          A_2*B*(Sabc-C_2) + 
          (4*C*A-B*B)*log( (2*A_2+BA+Sabc)/(BA+C_2) ) 
        )/(4*A_32);
    int precision = floor(length/(_MAX_Vel*0.01));
    cout<<"Precision: "<<precision<<endl;
    
    int precision0 = 10;
    double step = 1.0/precision;
   for(double i = 0.0; i < 1.0; i+=step){
        temp1 = p1*(1-i) + p2*i;
        temp2 = p2*(1-i) + p3*i;
        //cout<<"temp1: "<<temp1<<endl;
        //cout<<"temp2: "<<temp2<<endl;
        result = temp1*(1-i) + temp2*i;
        res.push_back(result);
   }
   ROS_WARN("Curve Generated!");
   return res;
}
int iterations = 0;
double sumDistance = 0;
double sumAngle = 0;
MyPoint* start = new MyPoint();
MyPoint* last = new MyPoint();
void clearParam(){
    iterations = 0;
    sumDistance = 0;
    sumAngle = 0;
    start = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());
    last = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());
}


int idx_ = 0;
bool mode = false;//1: straight  -1:turning 

void commandHelper(){ 
    /* 
    if(!_start_auto) return;
    //clearParam();
    double angleCommand, speedCommand;
    geometry_msgs::TwistStamped vel_msg;
    bool next = false;     
        
    if(!mode){
        Vector3d v1, v2;
        double theta, theta_ori;
        v2 = visTurning[idx_+1] - visTurning[idx_];
        if(idx_!=0){
            v1 = visTurning[idx_] - visTurning[idx_-1];   
        }else{
            v1 << cos(current_theta), sin(current_theta), 0;  
            //cout<<"Initial direction:"<<v1<<endl;
            //cout<<"First direction:" << v2<<endl;
        }
        //cout<<"v1:"<<v1<<"v2:"<<v2<<endl; 
        if(v1 != v2){ 
            theta_ori = v1.adjoint()*v2;
            theta = acos(theta_ori/(v1.norm()*v2.norm()));
        }else{
            theta = 0;
        }
        cout<<"turning angle:"<<theta<<endl;
        next = PIDCaculation(angleCommand, speedCommand, theta, 0);
        vel_msg = sendVelocity(speedCommand, angleCommand);
        _plan_vel_pub.publish(vel_msg);
        if(next) {
            !mode;
            clearParam();
        }
    }
        
    else{
        Vector3d distv = visTurning[idx_+1] - visTurning[idx_];
        double dist = distv.norm();
        next = PIDCaculation(angleCommand, speedCommand, 0, dist);
        vel_msg = sendVelocity(speedCommand, angleCommand);
        _plan_vel_pub.publish(vel_msg);
        if(next) {
            !mode;
            clearParam();
        }
    }
    if(next && !mode) idx_++;
    */

    //if(!_start_auto || Vel_Cmd.size()==0) return;
    if(!_start_auto) return;
    if(arrived) return;
     geometry_msgs::TwistStamped vel_msg;
    if(Vel_Cmd.size()==0 && inReplanning) {
        //cout<<"[CommandHelper] Inreplanning return."<<endl;
        finish_cmd = true;
        vel_msg = sendVelocity(0.0, 0.0); 
        _plan_vel_pub.publish(vel_msg);     
        return;
    }
    if(!inReplanning && finish_cmd && remainingTurning.size()==1){
        cout<<"[CommandHelper] Arrive Destination"<<endl;
        arrived = true;
        return;
    }
    
    if(msg_index==0 && Vel_Cmd.size()==0){
        bool moving = (fabs(_odom.twist.twist.linear.x) > 0.02 || fabs(_odom.twist.twist.linear.y > 0.02));
        //cout<<"Moving: "<<moving<<endl;
        if(moving) {
            //ROS_WARN("[CommandHelper] Wait for car stopping.");
            vel_msg = sendVelocity(0.0, 0.0); 
            _plan_vel_pub.publish(vel_msg);     
            return;
        }
        else{
            ROS_WARN("[CommandHelper] Stopped. Generate trajectory for remaining turnings.");
            if(!inReplanning)
                checkPosition(remainingTurning.back());
            return;
        }
    }
    //Vector3d cur_pt (_loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y, 0);
    //Vector3d dist = cur_pt - _first_pt;
   // if(msg_index == 0 && Vel_Cmd.size()==0 && dist.norm() > 0.1){
    // if(msg_index == 0 && Vel_Cmd.size()==0 && !moving){
    //     ROS_WARN("[CommandHelper] Stopped. Generate trajectory for remaining turnings.");
    //     if(!inReplanning)
    //         checkPosition(remainingTurning.front());
    //     return;
    // }
    
    idx_offset = round((_time_update_loop_odom-_loop_odom_time).toSec() * _frequency) + 4;
    //ROS_INFO("idx_offset: %i.", idx_offset);
    if(msg_index > Vel_Cmd.size() + idx_offset) {
        finish_cmd = true;
        if(msg_index == Vel_Cmd.size() + idx_offset+1){
            cout<<"[CommandHelper] Finish Command. Remaining turnings: "<<remainingTurning.size()<<endl;
        }
        if(inReplanning){
            msg_index++;
            return;
        }   
    }
    else{
        finish_cmd = false;
    }
    if(!inReplanning && msg_index < Ang_Cmd.size()-2 && Ang_Cmd[msg_index] != 0 && Ang_Cmd[msg_index+1] == 0){
        remainingTurning.erase(remainingTurning.begin());
    }
    Vector3d pt_;
    geometry_msgs::Vector3Stamped coord_msg;
    if(msg_index<planning_pt.size())
        pt_ = planning_pt[msg_index];
    else
        pt_ = planning_pt.back();
    coord_msg = sendCoordinate(pt_);
    _planning_coord_pub.publish(coord_msg); 

    if(msg_index<Vel_Cmd.size())
        vel_msg = sendVelocity(Vel_Cmd[msg_index], Ang_Cmd[msg_index]);
    else
        vel_msg = sendVelocity(0.0, 0.0); 
    _plan_vel_pub.publish(vel_msg);
    msg_index++;
    if(msg_index > idx_offset && msg_index < planning_pt.size()+idx_offset && Ang_Cmd[msg_index] == 0){ //&& Ang_Cmd[msg_index-idx_offset] == 0
        Vector3d plan_pt = planning_pt[msg_index-idx_offset];
        //ROS_INFO("planning coord: %f, %f", plan_pt(0), plan_pt(1));
        //ROS_INFO("actual pcoord: %f, %f", _loop_odom.pose.pose.position.x, _loop_odom.pose.pose.position.y);
        checkPosition(planning_pt[msg_index-idx_offset]);
    }
    
}


bool PIDCaculation(double &angleCommand, double &speedCommand, double targetAngle, double targetDistance)
{
    angleCommand = 0;
    speedCommand = 0;
    MyPoint* actual = new MyPoint(_odom.pose.pose.position.x, _odom.pose.pose.position.y, 2.0*asin(_odom.pose.pose.orientation.z), _odom.header.stamp);
    if (closeEnough(actual,targetDistance, targetAngle) == true)
    {
        ROS_INFO("GOAL ACHIEVED");
        //sendVelocity(0.0,0.0);
        angleCommand = 0;
        speedCommand = 0;
        return true;
    }
    if (iterations == 0)
    {
        start->x = actual->x;
        start->y = actual->y;
        start->time = actual->time;
        start->angle = actual->angle;
        last->x = actual->x;
        last->y = actual->y;
        last->time = actual->time;
        last->angle = actual->angle;
    }
    iterations++;

    //Calculation of action intervention.
    if (fabs(targetDistance) > TOLERANCE)
    {
        speedCommand = calculatePSD(actual,start->getDistance(actual)*copysign(1.0, targetDistance),start->getDistance(last)*copysign(1.0, targetDistance),targetDistance,F_KP,F_KD,F_KI,&sumDistance);
    }

    if (actual->angle-last->angle < -PI_)
    {
        actual->angle += 2*PI_;
    } 
    else if (actual->angle-last->angle > PI_)
    {
        actual->angle -= 2*PI_;
    }

    angleCommand = calculatePSD(actual,actual->angle-start->angle, last->angle-start->angle,targetAngle,R_KP,R_KD,R_KI,&sumAngle);

    //Saving position to last
    last->x = actual->x;
    last->y = actual->y;
    last->time = actual->time;
    last->angle = actual->angle;

    //Invoking method for publishing message
    //publishMessage(fmin(_MAX_Vel,angleCommand), fmin(_MAX_Vel,speedCommand));
    ROS_WARN("Start Publish Velovity. speedCommand: %d, angleCommand: %d", fmin(_MAX_Vel,speedCommand), fmin(_MAX_Vel,angleCommand));
    speedCommand = fmin(_MAX_Vel,speedCommand);
    angleCommand = fmin(_MAX_Vel,angleCommand);
    return false;
}

bool closeEnough(MyPoint* actual, double targetDistance, double targetAngle)
{
    double distance;
    distance = start->getDistance(actual)*copysign(1.0, targetDistance);
    if (fabs(distance-targetDistance) > TOLERANCE)
    {
        return false;
    }
    if (fabs(targetAngle - (actual->angle - start->angle)) > TOLERANCE_ANGLE &
        fabs(targetAngle - (actual->angle - start->angle) + 2*PI_) > TOLERANCE_ANGLE &
        fabs(targetAngle - (actual->angle - start->angle) - 2*PI_) > TOLERANCE_ANGLE)
    {
        return false;
    }
    return true;
}

double calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum)
{
    double speed = 0;
    double error = reference - actualValue;
    double previousError = reference - lastValue;
    double dt = actual->time.toSec() - last->time.toSec();
    double derivative = (error - previousError)/dt;
    *sum = *sum + error*dt;
    speed = kP*error + kD*derivative + kS*(*sum);
    return speed;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "trr_global_planner_node");
    ros::NodeHandle nh("~");

    nh.param("write_path", _WRITE_PATH, false);
    nh.param("read_path",  _READ_PATH,  false);

    nh.param("map/map_margin", _cloud_margin, 0.25);
    nh.param("map/resolution", _resolution,   0.2 );
    
    nh.param("map/x_size",         _x_size, 50.0);
    nh.param("map/y_size",         _y_size, 50.0);
    nh.param("map/z_size",         _z_size, 5.0 );
    
    nh.param("planning/rho_time",  _rho,       1.0);
    nh.param("planning/max_vel",   _MAX_Vel,   1.0);
    nh.param("planning/max_acc",   _MAX_Acc,   1.0);
    nh.param("planning/max_d_acc", _MAX_d_Acc, 1.0);
    nh.param("planning/wheel_radius", _wheel_r, 0.145);
    nh.param("planning/tolerance", TOLERANCE, 0.05);
    nh.param("planning/tolerance_angle", TOLERANCE_ANGLE, 0.05);


    nh.param("planning/max_inf_iter", _max_inf_iter,        10  );
    nh.param("planning/max_clu_iter", _max_clu_iter,        50  );
    nh.param("planning/rounding_radius", _rounding_radius,  1.5  );
    nh.param("planning/frequency", _frequency,              100.0  );
    nh.param("planning/safe_dist", _SAFE_DIST, 0.5);
    nh.param("optimization/min_order",  _minimize_order,    3.0 );
    nh.param("optimization/poly_order", _traj_order,        10  );
    
    nh.param("vis/vis_traj_width",     _vis_traj_width,     0.15);

    nh.param("your_file_path",         _your_file_path,     string("")  );

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _odom_sub = nh.subscribe( "odometry",  1, rcvOdometryCallBack);
    _loop_odom_sub = nh.subscribe( "loop_odometry",  1, rcvLoopOdometryCallBack);
    _auto_sub  = nh.subscribe( "auto",  1, rcvAutoModeCallBack );
    _joy_sub  = nh.subscribe( "joystick",  1, rcvJoyCallBack );
    _local_map_sub  = nh.subscribe( "local_map",  1,  rcvLocalPointCloudCallBack);  
    //_pid_sub = nh.subscribe("odometry",  1, &NodePID::messageCallback, nodePID);
    //ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodePID::messageCallback, nodePID);
    //carPlanning();
    // for visualization of the planning results
    _vis_traj_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_vis", 1);    
    _vis_grid_path_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1);
    _vis_inf_map_pub   = nh.advertise<sensor_msgs::PointCloud2>("inflation_map", 10);
    _vis_polytope_pub  = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_corridor_mesh", 1, true);
    _space_time_pub    = nh.advertise<quadrotor_msgs::SpatialTemporalTrajectory>("space_time_traj", 10); 
    _plan_vel_pub       = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 10);
    _planning_coord_pub = nh.advertise<geometry_msgs::Vector3Stamped>("planning_coord", 10);

    _map_lower << -_x_size/2.0, -_y_size/2.0, -0.1;
    _map_upper << +_x_size/2.0, +_y_size/2.0, _z_size;

    _poly_array_msg.header.frame_id = "/map";
    _vis_polytope_pub.publish(_poly_array_msg); 

    _pt_max_x = + _x_size / 2.0; _pt_min_x = - _x_size / 2.0;
    _pt_max_y = + _y_size / 2.0; _pt_min_y = - _y_size / 2.0; 
    _pt_max_z = + _z_size;       _pt_min_z = 0.0;
    
    _resolution = max(0.1, _resolution); // In case a too fine resolution crashes the CUDA code.
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)((_z_size + 0.1) * _inv_resolution);

    _path_finder  = new gridPathFinder(_max_x_id, _max_y_id, _max_z_id);
    _path_finder  -> initGridMap(_resolution, _map_lower, _map_upper);

    // _path_finder_local  = new gridPathFinder(_max_x_id*2, _max_y_id*2, _max_z_id*2);
    // _path_finder_local  -> initGridMap(0.1, _map_lower, _map_upper);
    
    _polyhedronGenerator->initialize(false, true, true, // method in generating corridor
        _max_x_id, _max_y_id, _max_z_id, _map_lower, _map_upper, _resolution, _inv_resolution, // map information
        _max_inf_iter, _max_clu_iter); // max infaltion/clustering num


    // if(_manual_path.size() != 0){
    //     carPlanning();
    // }

    ros::Rate rate(_frequency);
    ros::Rate check_rate(1);
    bool status = ros::ok();
    
    
    while(status){
       
        ros::spinOnce();
        commandHelper();
        replanning();
        
        /*
        if(Vel_Cmd.size()!=0 && msg_index<Vel_Cmd.size()){  
            if(msg_index < Ang_Cmd.size()-2 && Ang_Cmd[msg_index] != 0 && Ang_Cmd[msg_index+1] == 0){
                remainingTurning.erase(remainingTurning.begin());
            }
            
            geometry_msgs::Vector3Stamped coord_msg;
            if(msg_index<planning_pt.size())
                pt_ = planning_pt[msg_index];
            else
                pt_ = planning_pt.back();
            coord_msg = sendCoordinate(pt_);
            _planning_coord_pub.publish(coord_msg); 

            geometry_msgs::TwistStamped vel_msg;
            if(msg_index<Vel_Cmd.size())
                vel_msg = sendVelocity(Vel_Cmd[msg_index], Ang_Cmd[msg_index]);
            else
                vel_msg = sendVelocity(0.0, 0.0); 
            _plan_vel_pub.publish(vel_msg);
            //ROS_INFO("vel cmd: %f, ang cmd: %f, remaining turns: %i.", Vel_Cmd[msg_index], Ang_Cmd[msg_index], remainingTurning.size());
            //ROS_INFO("planning coord: %f, %f", pt_(0), pt_(1));
            //ROS_INFO("actual pcoord: %f, %f", _odom.pose.pose.position.x, _odom.pose.pose.position.y);
            //cout<<endl;
            idx_offset = (_time_update_odom-_odom_time).toSec() * _frequency + 3;
            if(!self_replan && msg_index > idx_offset && msg_index < planning_pt.size()+idx_offset){
                ROS_INFO("planning coord: %f, %f", pt_(0), pt_(1));
                ROS_INFO("actual pcoord: %f, %f", _odom.pose.pose.position.x, _odom.pose.pose.position.y);
                checkPosition(planning_pt[msg_index-idx_offset]);
            }
            msg_index++;
            
            if(msg_index>=Vel_Cmd.size()){
                ROS_WARN("Finish sending command.");
                finish_cmd = true;
                if(self_replan){
                    msg_index = 0;
                    Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
                    Vector3d current_pt(_odom.pose.pose.position.x, _odom.pose.pose.position.y, 0);
                    remainingTurning.insert(remainingTurning.begin(), current_pt);
                    trajectoryGeneration(remainingTurning); 
                    self_replan = false;
                }
                if(inReplanning){
                    msg_index = 0;
                    Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
                    collision_status = checkCollision();
                    replanning();
                }
            }
        }*/
         
        status = ros::ok();
        rate.sleep();
    }

    delete _path_finder;
    //delete _path_finder_local;
    return 0;
}

geometry_msgs::TwistStamped sendVelocity(double linear, double angular){
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp       = ros::Time::now();
    cmd_vel.header.frame_id    = "/trr_global_planner";
    cmd_vel.twist.linear.x = linear;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.linear.z = 0.0;
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = angular;
    
    return cmd_vel;
    //_plan_vel_pub.publish(cmd_vel);

}

geometry_msgs::Vector3Stamped sendCoordinate(Vector3d coord){
    geometry_msgs::Vector3Stamped planning_coord;
    planning_coord.header.stamp       = ros::Time::now();
    planning_coord.header.frame_id    = "/trr_global_planner";
    planning_coord.vector.x = coord(0);
    planning_coord.vector.y = coord(1);
    planning_coord.vector.z = coord(2);
    return planning_coord;
}

geometry_msgs::Point Vector2Point(Vector3d vec)
{
    geometry_msgs::Point pt;
    pt.x = vec(0);   
    pt.y = vec(1);   
    pt.z = vec(2);   

    return pt;
}

void clearVisualization(int grid_path_mk_num, int traj_mk_iter_num)
{
    // 1. Clear the Grid Path :
    visualization_msgs::MarkerArray grid_vis; 
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "trr_global_planner/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::DELETE;
    
    for(int i = 0; i < grid_path_mk_num; i++)
    {
        mk.id = i;
        grid_vis.markers.push_back(mk);
    }

    _vis_grid_path_pub.publish(grid_vis);

    // 2. Clear the Bezier Curves
    cout<<"bezier curves visualization num: "<<_vis_iter_num<<endl;
    for(int i = 0; i < _vis_iter_num; i++)
    {
        visualization_msgs::Marker traj_vis;

        traj_vis.header.stamp       = ros::Time::now();
        traj_vis.header.frame_id    = "map";
        traj_vis.ns = "trr_global_planner/trajectory" + to_string(i);
        traj_vis.id = 0;
        traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        traj_vis.action = visualization_msgs::Marker::DELETE;

        for(int k = 0; k < 100; k++)
            _vis_traj_pub.publish(traj_vis);
    }

    // 3. Clear the polyhedrons
    _poly_array_msg.polyhedrons.clear();
    _poly_array_msg.ids.clear();
    _vis_polytope_pub.publish(_poly_array_msg);
}

void visCorridor( decomp_ros_msgs::PolyhedronArray & poly_array_msg )
{
    _vis_polytope_pub.publish(poly_array_msg);
}

void visBezierTrajectory(vector<Vector3d> Points, int iter_id)
{   
    visualization_msgs::Marker traj_vis;

    _vis_iter_num ++;
    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory" + to_string(iter_id);
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    //traj_vis.color.r = min(1.0, (iter_id + 1) / 5.0);
    traj_vis.color.r = 0.0;
    traj_vis.color.g = 0.0;//max(0.0, 1 - rgb / 5.0);
    traj_vis.color.b = 0.0;

    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;
    for(auto vertex : Points){
        pt.x = vertex[0];
        pt.y = vertex[1];
        pt.z = vertex[2];
        traj_vis.points.push_back(pt);
    }

    _vis_traj_pub.publish(traj_vis);

    // int segment_num  = polyCoeff.rows();
    // for(int i = 0; i < segment_num; i++ ){
    //     for (double t = 0.0; t < 1.0; t += 0.01 / time(i), count += 1){
    //         state = _bezier_basis->getPosFromBezier( polyCoeff, t, i );
    //         cur(0) = pt.x = time(i) * state(0);
    //         cur(1) = pt.y = time(i) * state(1);
    //         cur(2) = pt.z = time(i) * state(2);
    //         traj_vis.points.push_back(pt);

    //         if (count) traj_len += (pre - cur).norm();
    //         pre = cur;
    //     }
    // }

    _vis_traj_pub.publish(traj_vis);
}

void visLineSegmentTrajectory(vector<Vector3d> Points)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory_line";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
    
    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;


    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

   
    geometry_msgs::Point pt;

    for(auto vertex : Points){
        pt.x = vertex[0];
        pt.y = vertex[1];
        pt.z = vertex[2];
        traj_vis.points.push_back(pt);
    }

    _vis_traj_pub.publish(traj_vis);
}

void visLineSegment(vector<Vector3d> Points)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory_line";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::LINE_LIST;
    
    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;

    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

   
    geometry_msgs::Point pt;
    for(int i = 0; i < Points.size()-1; i+=2){
        Vector3d Point1 = Points[i];
        Vector3d Point2 = Points[i+1];
        pt.x = Point1[0];
        pt.y = Point1[1];
        pt.z = Point1[2];
        traj_vis.points.push_back(pt);
        pt.x = Point2[0];
        pt.y = Point2[1];
        pt.z = Point2[2];
        traj_vis.points.push_back(pt);
    }
    _vis_traj_pub.publish(traj_vis);
}


void visPoint(vector<Vector3d> Points) 
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory_point";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::POINTS;
    
    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = 0.2;
    traj_vis.scale.y = 0.2;
    traj_vis.scale.z = 0.2;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    traj_vis.color.r = 0.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 1.0;

    geometry_msgs::Point pt;
    for(auto vertex : Points){
        pt.x = vertex[0];
        pt.y = vertex[1];
        pt.z = vertex[2];
        traj_vis.points.push_back(pt);
    }

    _vis_traj_pub.publish(traj_vis);
}

void visGridPath( )
{   
    visualization_msgs::MarkerArray grid_vis; 
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "trr_global_planner/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    
    int idx = 0;
    for(int i = 0; i < int(_manual_path.size()); i++)
    {
        mk.id = idx;
        mk.pose.position.x = _manual_path[i](0); 
        mk.pose.position.y = _manual_path[i](1); 
        mk.pose.position.z = _manual_path[i](2);  

        mk.scale.x = _resolution;
        mk.scale.y = _resolution;
        mk.scale.z = _resolution;

        idx ++;
        grid_vis.markers.push_back(mk);
    }

    _vis_grid_path_pub.publish(grid_vis);
}