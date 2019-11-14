
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <global_planner/utils/data_type.h>
#include <global_planner/utils/bezier_base.h>

using namespace std;
using namespace Eigen;

struct TurningNode;
typedef TurningNode* TurningNodePtr;

struct TurningNode
{     
   double radius, longrad;
   Eigen::Vector3d centerCoord, preCoord, nextCoord;
   Eigen::Vector3d preVec, nextVec;
   double wider; // x direction longer: 1  y direction longer: -1 

   TurningNode(Eigen::Vector3d _centerCoord, Eigen::Vector3d _preVec, 
                Eigen::Vector3d _nextVec, double _radius, double _longrad, double _wider)
   {  
     wider = _wider;
     radius = _radius;
     longrad = _longrad;
     centerCoord = _centerCoord;
     preVec = _preVec;
     nextVec = _nextVec;
     preCoord = centerCoord - preVec * radius;
     nextCoord = centerCoord + nextVec * radius;

   }
   TurningNode(Eigen::Vector3d _centerCoord, Eigen::Vector3d _preVec, 
                Eigen::Vector3d _nextVec, double _radius)
   {  
     wider = 1;
     radius = _radius;
     longrad = _radius;
     centerCoord = _centerCoord;
     preVec = _preVec;
     nextVec = _nextVec;
     preCoord = centerCoord - preVec * radius;
     nextCoord = centerCoord + nextVec * radius;

   }
   TurningNode(Eigen::Vector3d _centerCoord, double _radius, double _longrad, double _wider)
   {  
     wider = _wider;
     radius = _radius;
     longrad = _longrad;
     centerCoord = _centerCoord;
   }

   TurningNode(){};
   
   ~TurningNode(){};
};

class simpleTrajOptimizer 
{
    private:
        double obj;

    public:
        simpleTrajOptimizer(){}
        ~simpleTrajOptimizer(){}

        void lineSegmentGeneration( 
        const decomp_cvx_space::FlightCorridor &corridor,
        vector<Eigen::Vector3d> &trrSegPts,
        vector<TurningNode> &turningPts,
        const double maxVel, 
        const double maxAcc);

        void turningPtGeneration(
            vector<Eigen::Vector3d> &trrSegPts,
            vector<TurningNodePtr> &List
        );
        //void findVertex(double wider, double long, double r, Vector3d center, Vector3d& lb, Vector3d& ru);
        int lineCross(Vector3d pt1, Vector3d pt2, TurningNode turn);
        int IntervalOverlap(double x1, double x2, double x3, double x4){
            double t;
            if (x3 > x4){
                t = x3;
                x3 = x4;
                x4 = t;
            }
            if (x3 > x2 || x4 < x1)
                return 0;
            else
                return 1;
        };
        double getObjective()
        {
            return obj;
        };
};


