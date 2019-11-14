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
#include <global_planner/temporal_optimizer.h>
#include <global_planner/spatial_optimizer.h>
#include <global_planner/simple_optimizer.h>
#include <std_msgs/Bool.h>
using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _vis_traj_width;
double _x_size, _y_size, _z_size, _resolution, _inv_resolution;
double _cloud_margin, _minimize_order, _rounding_radius;
double _MAX_Vel, _MAX_Acc, _MAX_d_Acc, _wheel_r, _frequency, _SAFE_DIST;
int    _traj_order, _max_inf_iter, _max_clu_iter;
string _your_file_path;
double current_theta;
// useful global variables
bool _has_odom  = false;
bool _has_map   = false;
bool _has_traj  = false;
bool _start_auto;
Vector3d _start_pt,  _end_pt;
Vector3d _map_lower, _map_upper;
bool cmd_send = false;
double _pt_max_x, _pt_min_x, _pt_max_y, _pt_min_y, _pt_max_z, _pt_min_z;
double _rho;
double cur_theta;

int _max_x_id, _max_y_id, _max_z_id;
int _traj_id = 1;
int _vis_iter_num = 0;
int msg_index = 0;
vector<Vector3d> _manual_path, trrSegPts, visTurning,remainingTurning;
decomp_ros_msgs::PolyhedronArray _poly_array_msg;
vector<double> rList, longList;
vector<TurningNode> turningPts;
vector<double> Vel_Cmd, Ang_Cmd;
vector<Vector3d> planning_pt;
// ros related
ros::Subscriber _map_sub, _odom_sub, _auto_sub, _local_map_sub;
ros::Publisher _vis_polytope_pub, _vis_traj_pub, _vis_grid_path_pub, _vis_inf_map_pub, _plan_vel_pub, _planning_coord_pub;
ros::Publisher _space_time_pub;

ros::Time _odom_time, _traj_time_start, _traj_time_final, _time_update_odom;
nav_msgs::Odometry _odom;

// useful object
Bernstein      * _bezier_basis             = new Bernstein(); 
gridPathFinder * _path_finder              = new gridPathFinder();
polyhedronGenerator * _polyhedronGenerator = new polyhedronGenerator();

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void rcvOdometryCallBack(const nav_msgs::Odometry odom);
void rcvAutoModeCallBack(const std_msgs::Bool auto_mode);

void trajPlanning();
void carPlanning();
void trajectoryGeneration(vector<Vector3d> turning);
bool isNext(Vector3d coord_1, Vector3d coord_2);


void visCorridor( decomp_ros_msgs::PolyhedronArray & poly_array_msg );
void visGridPath();
void visBezierTrajectory(vector<Vector3d> Points, int iter_id);
void visFinalBezierTrajectory(MatrixXd polyCoeff, VectorXd time);
void visPoint(vector<Vector3d> Points);
void visLineSegmentTrajectory(vector<Vector3d> Points);
void visLineSegment(vector<Vector3d> Points);
void clearVisualization(int grid_path_mk_num, int traj_mk_iter_num);

void trajCaculation(double dist, double frequency, vector<double> &vec_Vel, vector<double>  &vec_Dist);
vector<Vector3d> bezierGeneration(TurningNode turning);
geometry_msgs::TwistStamped sendVelocity(double linear, double angular);
geometry_msgs::Vector3Stamped sendCoordinate(Vector3d coord);
quadrotor_msgs::SpatialTemporalTrajectory getSpaceTimeTraj(const timeAllocator * time_allocator, const MatrixXd & bezier_coeff, const VectorXd & range );
fstream path_record;
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{    
    _end_pt << wp.poses[0].pose.position.x,
               wp.poses[0].pose.position.y,
               wp.poses[0].pose.position.z;
}

bool _WRITE_PATH, _READ_PATH, _INIT_PLAN;
void rcvAutoModeCallBack(const std_msgs::Bool auto_mode)
{
    //wheel_driver::WheelEncoderMsg _wheel_data = wheel_data;
    //int32_t  left = _wheel_data.left_encoding;
    //int32_t right = _wheel_data.right_encoding;
    //if(!_INIT_PLAN && left != 0 && right != 0)
    _start_auto = auto_mode.data;
    if(_start_auto)
    {
        //_INIT_PLAN = true;
        ROS_WARN("[trr_global_planner] Enter in autonomous mode");
        if (_WRITE_PATH)
        {
            for(auto pt: _manual_path)
                path_record<<pt(0)<<" "<<pt(1)<<" "<<pt(2)<<"\n";
        }
                
        if (_READ_PATH)
        {
            std::ifstream infile(_your_file_path);
            double x, y, z;

            while (infile >> x >> y >> z){
                Vector3d pt(x, y, z);
                _manual_path.push_back(pt);
            }

            _polyhedronGenerator->corridorIncreGeneration(_manual_path, _poly_array_msg);
        }

        if(_manual_path.size() == 0) return;
        _start_pt = _manual_path.front();
        Vector3d current_pt(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);
        Vector3d init_dist_vec = _start_pt - current_pt;
        if(init_dist_vec.norm()>2*_MAX_Vel){
            ROS_WARN("[trr_global_planner] Too far away from start point.");
            return;
        }
        visTurning.push_back(current_pt);
        current_theta = 2 * atan2(_odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w);
        //visGridPath( );
        //trajPlanning();
        //rospy.sleep(2);
        carPlanning(); 
    }
    
}

Vector3i _pose_idx, _pose_idx_lst;
void rcvOdometryCallBack(const nav_msgs::Odometry odom)
{
    _odom = odom;
    _odom_time = odom.header.stamp;
    _time_update_odom = ros::Time::now();

    _has_odom = true;

    Vector3d pos, pos_round, vel, direction_norm, sendVelocity;
    pos(0)  = odom.pose.pose.position.x;
    pos(1)  = odom.pose.pose.position.y;
    pos(2)  = odom.pose.pose.position.z;  
    vel(0)  = odom.twist.twist.linear.x;
    vel(1)  = odom.twist.twist.linear.y;
    vel(2)  = odom.twist.twist.linear.z;  

    direction_norm = vel / vel.norm();
    cur_theta = atan2(_vel_plan.twist.linear.y, _vel_plan.twist.linear.x);

    _pose_idx = _path_finder->coord2gridIndex(pos);
    pos_round = _path_finder->gridIndex2coord(_pose_idx);
    vector<Vector3d> coord_set;
    
    if(_start_auto){
        if(!checkCollision() ){
            geometry_msgs::TwistStamped vel_msg;
            if(msg_index<Vel_Cmd.size())
                vel_msg = sendVelocity(Vel_Cmd[msg_index], Ang_Cmd[msg_index]);
            else
                vel_msg = sendVelocity(0.0, 0.0); 
            _final_vel_pub.publish(vel_msg);
            msg_index++;
        }
        else{
            Replanning();
        }

    }
    
    else{
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

        if( _polyhedronGenerator->corridorIncreGeneration(coord_set, _poly_array_msg) == 1 )
            visCorridor(_poly_array_msg);
    }
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
  //if( _has_map ) return;

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

                    // set in obstacle points
                    _path_finder->setObs(inf_x, inf_y, inf_z);
                    

                    // rounding for visualizing the grid map
                    // Vector3d coor_round = _path_finder->gridIndex2coord( idx_inf );
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

    // pcl::toROSMsg(cloud_inf, map_inflation);
    // map_inflation.header.frame_id = "/map";
    // _vis_inf_map_pub.publish(map_inflation);

    //_has_map = true;
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if( _has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_inf;
    pcl::fromROSMsg(pointcloud_map, cloud);
    sensor_msgs::PointCloud2 map_inflation;

    if( (int)cloud.points.size() == 0)
        return;

    pcl::PointXYZ pt, pt_inf;

    int inf_step   = round(_cloud_margin * _inv_resolution);
    int inf_step_z = max(1, inf_step / 2);
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

bool checkCollision()
{
    if(rotating) return false;
    double dist = _SAFE_DIST;
    Vector3d check_pt, check_pt_round;
    Vector3i check_idx, pos_idx;
    vector<Vector3d> local_path;
    pos_idx = _path_finder->coord2gridIndex(pos);
    check_pt(0) = pos(0) + dist * direction_norm(0);
    check_pt(1) = pos(0) + dist * direction_norm(0);
    check_pt(2) = pos(0) + dist * direction_norm(0);
    check_idx = _path_finder->coord2gridIndex(check_pt);
    check_pt_round = _path_finder->gridIndex2coord(check_idx);

    // _path_finder->AstarSearch(pos_round, check_pt_round);
    // local_path = _path_finder->getPath();
    // _path_finder->resetMap();

    //simple version without considering obstacles near turning pt
    // if(_path_finder->IndexQuery(check_idx) > 0)
    //     return true;
    // else 
    //     return false;

    int minx = min(pos_idx(0), check_idx(0));
    int miny = min(pos_idx(1), check_idx(1));
    int minz = min(pos_idx(2), check_idx(2));
    int maxx = max(pos_idx(0), check_idx(0));
    int maxy = max(pos_idx(1), check_idx(1));
    int maxz = max(pos_idx(2), check_idx(2));

    for(int i = minx; i <= maxx; i++){
        for(int j = miny; j <= maxy; j++){
            for(int k = minz; j <= maxz; j++){
                if(_path_finder->IndexQuery(i,j,k) > 0)
                    return true;
            }
        }
    }

    return false;    
}
void Replanning(){
    //going straight
    if(odom.twist.twist.angular.z < 0.1){
        double next_vel = Vel_Cmd[msg_index];
        Vel_Cmd.clear()
         geometry_msgs::TwistStamped vel_msg;
         double delta_v = _MAX_d_Acc / _frequency;
         vel_msg = sendVelocity(next_vel-delta_v, Ang_Cmd[msg_index]);
         _plan_vel_pub.publish(vel_msg);
    }
    //stop
    if(odom.twist.twist.linear.x == 0 && odom.twist.twist.linear.y == 0){
        //turn theta clockwise.
        //....
        if(!checkCollision()){
            //remainingTurning.insert(remainingTurning.begin(), current_pt);
        }
    }
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
    _end_pt   = _manual_path.back();

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
    visPoint(visTurning);
    //visLineSegment(visTurning);
    visLineSegmentTrajectory(visTurning);
    //remainingTurning = visTurning;
    trajectoryGeneration(visTurning);
    ROS_WARN("Command Velocity Numbers: %i, Angular Numbers: %i, Coord Numbers: %i", Vel_Cmd.size(), Ang_Cmd.size(), planning_pt.size());
    // Vector3d linear, angular;
    // geometry_msgs::TwistStamped vel_msg = sendVelocity(linear, angular);
    // _plan_vel_pub.publish(vel_msg);

    // for(int i = 0; i < turningPts.size(); i++){
    //     TurningNode curTurn = turningPts[i];
    //     //ROS_WARN("Before generate curve!");
    //     vector<Vector3d> curve = bezierGeneration(curTurn);
    //     visBezierTrajectory(curve, i);
    //     //visLineSegmentTrajectory(curve);
    // }

    delete line_optimizer;
    _has_traj = true;
}
void trajectoryGeneration(vector<Vector3d> turning){  
    //double _Vel = _MAX_Vel;
    //double _Acc = _MAX_Acc;
    //double _Dcc = _MAX_d_Acc;
    //double _Rad = _MAX_Rad; 
    // double _Vel = 1.0;
    // double _Acc = 1.0;
    // double _Dcc = 0.5;
    // double _Rad = 1.0;
    //double wheel_r = 0.1;
    Vel_Cmd.clear(); Ang_Cmd.clear(); planning_pt.clear();
    Vector3d distv, dir_norm;
    double frequency = _frequency;
    double vel, theta, theta_ori;
    double dist, ccw;
    Vector3d v1, v2, pt;
    vector<double> tempVel, tempDist;
    vector<Vector3d> coordinate;
    remainingTurning = turning;
    for(int i = 0; i < turning.size()-1; i++){
        vel = 0;        
        //Turning Part
        v2 = turning[i+1] - turning[i];
        if(i!=0){
            v1 = turning[i] - turning[i-1];   
        }else{
            v1 << cos(current_theta), sin(current_theta), 0;  
            cout<<"Initial direnction:"<<v1<<endl;
            cout<<"First direction:" << v2<<endl;
        }
            
            //theta = (v1.adjoint()*v2)/(v1.norm()*v2.norm());
            theta_ori = v1.adjoint()*v2;
            theta = acos(theta_ori/(v1.norm()*v2.norm()));
            ccw = v1(0)*v2(1) - v1(1)*v2(0);
            dist = theta * _wheel_r;
            theta = (ccw > 0 ? 1 : -1)*theta;
            cout<<"turning angle:"<<theta<<endl;
            trajCaculation(dist, frequency, tempVel, tempDist);
            for(auto vel_temp : tempVel){
                Vel_Cmd.push_back(vel_temp);
                Ang_Cmd.push_back(theta);
                planning_pt.push_back(turning[i]);
            }
            tempVel.clear();
            tempDist.clear();
        //}

        //Line Part 
        distv = turning[i+1] - turning[i];
        dist = distv.norm();
        dir_norm = distv / dist;
        //cout<<"Distance:"<<dist<<endl;
        trajCaculation(dist, frequency, tempVel, tempDist);
        for(auto vel_temp : tempVel){
            Vel_Cmd.push_back(vel_temp);
            Ang_Cmd.push_back(0);
        }
        for(auto d : tempDist){
            pt = turning[i] + d * dir_norm;
            planning_pt.push_back(pt);
        }
        tempVel.clear();
        tempDist.clear();
        remainingTurning.erase(remainingTurning.begin());
    }

    //visPoint(planning_pt);
}
void trajCaculation(double dist, double frequency, vector<double> &vec_Vel, vector<double>  &vec_Dist){
    double freq_inv = 1/frequency;
    double v0, t0, t1, t2, s0, vel_cur;
    //double _Vel = _MAX_Vel;
    //double _Acc = _MAX_Acc;
    //double _Dcc = _MAX_d_Acc;
    //double _Rad = _MAX_Rad; 
    double _Vel = 1.0;
    double _Acc = 1.0;
    double _Dcc = 0.5;
    double vel = 0;
    double distunit = 0;
    //vector<double> vec_Vel, vec_Dist;
    s0 = 0.5 * _Vel * _Vel * (1/_Dcc + 1/_Acc);
    if(dist > s0){
        cout<<"Can Achieve Set Max Speed."<<endl;
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
        cout<<"Can't achieve! Max speed is:"<<v0<<endl;
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
void checkPosition(Vector3d targetPt){
    Vector3d current_pt(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);
    Vector3d dist_vec = targetPt - current_pt;
    double current_dist = dist_vec.norm();
    if(current_dist > 4*_wheel_r){
        ROS_WARN("Left Trajectory. Replanning...");
        //msg_index = 0;
        //remainingTurning.insert(remainingTurning.begin(), current_pt);
        //trajectoryGeneration(remainingTurning);   
    }
    else
        ROS_WARN("On Trajectory.");
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
    nh.param("planning/wheel_radius", _wheel_r, 0.1);

    nh.param("planning/max_inf_iter", _max_inf_iter,        10  );
    nh.param("planning/max_clu_iter", _max_clu_iter,        50  );
    nh.param("planning/rounding_radius", _rounding_radius,  1.5  );
    nh.param("planning/frequency", _frequency,              10.0  );
      nh.param("planning/safe_dist", _SAFE_DIST,            1.0);
    nh.param("optimization/min_order",  _minimize_order,    3.0 );
    nh.param("optimization/poly_order", _traj_order,        10  );
    
    nh.param("vis/vis_traj_width",     _vis_traj_width,     0.15);

    nh.param("your_file_path",         _your_file_path,     string("")  );

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _odom_sub = nh.subscribe( "odometry",  1, rcvOdometryCallBack);
    _auto_sub  = nh.subscribe( "auto",  1, rcvAutoModeCallBack );
    _local_map_sub  = nh.subscribe( "local_map",  1,  rcvLocalPointCloudCallBack);
    //carPlanning();
    // for visualization of the planning results
    _vis_traj_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_vis", 1);    
    _vis_grid_path_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1);
    _vis_inf_map_pub   = nh.advertise<sensor_msgs::PointCloud2>("inflation_map", 10);
    _vis_polytope_pub  = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_corridor_mesh", 1, true);
    _space_time_pub    = nh.advertise<quadrotor_msgs::SpatialTemporalTrajectory>("space_time_traj", 10); 
    _plan_vel_pub       = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 10);
    _planning_coord_pub = nh.advertise<geometry_msgs::Vector3Stamped>("planning_coord", 10);

    _map_lower << -_x_size/2.0, -_y_size/2.0, 0.0;
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
    _max_z_id = (int)(_z_size * _inv_resolution);

    _bezier_basis = new Bernstein(_minimize_order);
    _bezier_basis ->setFixedOrder(_traj_order);

    _path_finder  = new gridPathFinder(_max_x_id, _max_y_id, _max_z_id);
    _path_finder  -> initGridMap(_resolution, _map_lower, _map_upper);
    
    _polyhedronGenerator->initialize(false, true, true, // method in generating corridor
        _max_x_id, _max_y_id, _max_z_id, _map_lower, _map_upper, _resolution, _inv_resolution, // map information
        _max_inf_iter, _max_clu_iter); // max infaltion/clustering num

    if(_WRITE_PATH){
        path_record.open(_your_file_path);
    }

    // if(_manual_path.size() != 0){
    //     carPlanning();
    // }

    ros::Rate rate(_frequency);
    ros::Rate check_rate(1);
    bool status = ros::ok();
    int i = 0;
    while(status){
        // if(Vel_Cmd.size()!=0){     
        //     geometry_msgs::Vector3Stamped coord_msg;
        //     if(i<planning_pt.size())
        //         coord_msg = sendCoordinate(planning_pt[msg_index]);
        //     else
        //         coord_msg = sendCoordinate(planning_pt.back());
        //     _planning_coord_pub.publish(coord_msg); 

        //     geometry_msgs::TwistStamped vel_msg;
        //     if(msg_index<Vel_Cmd.size())
        //         vel_msg = sendVelocity(Vel_Cmd[msg_index], Ang_Cmd[msg_index]);
        //     else
        //         vel_msg = sendVelocity(0.0, 0.0); 
        //     _plan_vel_pub.publish(vel_msg);
            
        //     //if(msg_index != 0)
        //     //    checkPosition(planning_pt[msg_index-1]);
            
        //     msg_index++;
        // }
        ros::spinOnce();  
        status = ros::ok();
        rate.sleep();
    }
    // while(status){
    //     checkPosition(planning_pt[i-1]);
    //     status = ros::ok();
    //     check_rate.sleep();
    // }

    delete _bezier_basis;
    delete _path_finder;

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

quadrotor_msgs::SpatialTemporalTrajectory getSpaceTimeTraj(const timeAllocator * time_allocator, const MatrixXd & bezier_coeff, const VectorXd & range )
{
    quadrotor_msgs::SpatialTemporalTrajectory space_time_traj;
    space_time_traj.action = quadrotor_msgs::OptimalTimeAllocator::ACTION_ADD;
    space_time_traj.header.frame_id = "/trr_global_planner";
    space_time_traj.trajectory_id = _traj_id;

    space_time_traj.header.stamp = _traj_time_start; 
    space_time_traj.start_time   = _traj_time_start; 
    space_time_traj.final_time   = _traj_time_final; 

    int seg_num = time_allocator->seg_num;
    space_time_traj.s_step = time_allocator->s_step; 
    space_time_traj.K_max  = time_allocator->K_max;

    for (int i = 0; i < seg_num; i++ )
    {     
        space_time_traj.K.push_back(time_allocator->K(i));

        for(int j = 0; j < time_allocator->K(i) + 1; j++ )
        {   
            if( j < time_allocator->K(i) )
            {
                space_time_traj.a.push_back( time_allocator->a(i, j) );
                space_time_traj.time.push_back    ( time_allocator->time(i, j) );
                space_time_traj.time_acc.push_back( time_allocator->time_acc(i, j) );
            }

            space_time_traj.b.push_back( time_allocator->b(i, j) );
            space_time_traj.s.push_back( time_allocator->s(i, j) );
        }
    }

    /*  stack the spatial curve  */
    seg_num = range.size();
    space_time_traj.num_segment = seg_num;

    for(int i = 0; i < seg_num; i++ )
    {    
        int poly_num1d = _traj_order + 1;
        for(int j =0; j < poly_num1d; j++)
        { 
            space_time_traj.coef_x.push_back(bezier_coeff(i,                  j));
            space_time_traj.coef_y.push_back(bezier_coeff(i,     poly_num1d + j));
            space_time_traj.coef_z.push_back(bezier_coeff(i, 2 * poly_num1d + j));
        }
        space_time_traj.range.push_back(range(i));
    }

    space_time_traj.start_yaw = 0.0;
    space_time_traj.final_yaw = 0.0;
    space_time_traj.trajectory_id = _traj_id;

    return space_time_traj;
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

void visFinalBezierTrajectory(MatrixXd polyCoeff, VectorXd time)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory_final";
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
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;

    int segment_num  = polyCoeff.rows();
    for(int i = 0; i < segment_num; i++ ){
        for (double t = 0.0; t < 1.0; t += 0.01 / time(i), count += 1){
            state = _bezier_basis->getPosFromBezier( polyCoeff, t, i );
            cur(0) = pt.x = time(i) * state(0);
            cur(1) = pt.y = time(i) * state(1);
            cur(2) = pt.z = time(i) * state(2);
            traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }

    _vis_traj_pub.publish(traj_vis);
}

void visTurningCurve(MatrixXd polyCoeff, VectorXd time, vector<TurningNode> turning)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "trr_global_planner/trajectory_final";
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
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;

    int segment_num  = 0.5 * turning.size() - 1;
    for(int i = 0; i < segment_num; i++ ){
        for (double t = 0.0; t < 1.0; t += 0.01 / time(i), count += 1){
            state = _bezier_basis->getPosFromBezier( polyCoeff, t, i );
            cur(0) = pt.x = time(i) * state(0);
            cur(1) = pt.y = time(i) * state(1);
            cur(2) = pt.z = time(i) * state(2);
            traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }

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
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
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