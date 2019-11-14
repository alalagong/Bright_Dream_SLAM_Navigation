#include <global_planner/simple_optimizer.h>
using namespace std;    
using namespace Eigen;

void simpleTrajOptimizer::lineSegmentGeneration( 
        const decomp_cvx_space::FlightCorridor &corridor,
        vector<Vector3d> &trrSegPts,
        vector<TurningNode> &turningPts,
        const double maxVel, 
        const double maxAcc) 
{   
#define ENFORCE_VEL 0
#define ENFORCE_ACC 0 

    vector<decomp_cvx_space::Polytope> polyhedrons  = corridor.polyhedrons;
    int segment_num  = polyhedrons.size();
    vector<double> rList, longList, widerList;
    Vector3d start_pt = trrSegPts.front();
    Vector3d end_pt = trrSegPts.back();
    trrSegPts.pop_back();
    Vector3d center;
    double longrad, radius, wider;
    decomp_cvx_space::Polytope curPoly, nextPoly;
    for(int i = 0; i < segment_num-1; i++){
        vector<double> x_coord, y_coord;
        vector<Vector3d> tempPts;
        curPoly = polyhedrons[i];
        nextPoly = polyhedrons[i+1];
        //center = curPoly.vertices[0];
        tempPts.push_back(curPoly.vertices[1]); tempPts.push_back(curPoly.vertices[4]);
        tempPts.push_back(nextPoly.vertices[1]); tempPts.push_back(nextPoly.vertices[4]);
        for(auto pt : tempPts){
            x_coord.push_back(pt(0));
            y_coord.push_back(pt(1));
        }
        sort(x_coord.begin(), x_coord.end());
        sort(y_coord.begin(), y_coord.end());
        center(0) = (x_coord[1] + x_coord[2]) * 0.5; center(1) = (y_coord[1] + y_coord[2]) * 0.5; center(2) = 0;
        wider = (fabs(x_coord[1] - x_coord[2]) > fabs(y_coord[1] - y_coord[2]))? 1 : -1;
        radius = 0.5 * min(fabs(x_coord[1] - x_coord[2]), fabs(y_coord[1] - y_coord[2]));
        longrad = 0.5 * max(fabs(x_coord[1] - x_coord[2]), fabs(y_coord[1] - y_coord[2]));
        rList.push_back(radius);
        longList.push_back(longrad);
        widerList.push_back(wider);
        trrSegPts.push_back(center);   
    }
    trrSegPts.push_back(end_pt);
    cout<<"Original trrseg number: "<< trrSegPts.size()<<endl;

    vector<Vector3d> trrPtsv1;
    vector<double> rListv1, lListv1, wListv1;
    Vector3d preVec, nextVec, preVecNorm, nextVecNorm;
    int curNum = rList.size();
    for(int i = 0; i < curNum; i++){
        if(i != 0 ){
            preVec = trrSegPts[i+1] - trrPtsv1.back();
            if(rListv1.back()>rList[i] && preVec.norm()<rListv1.back()- rList[i]){
                trrPtsv1.pop_back();
                rListv1.pop_back();
                lListv1.pop_back();
                wListv1.pop_back();
                cout<<"Abandon Last Turning Pt."<<endl;
            }     
            else if(rListv1.back()<rList[i] && preVec.norm()<rList[i]-rListv1.back()){
                cout<<"Abandon Current Turning Pt."<<endl;
                continue;
            }
            /* 
            //For Curves Turning    
            else if(preVec.norm()<rListv1.back()+rList[i] && preVec.norm()>min(rList[i], rListv1.back())){
                cout<<"Reform a new Turning PT."<<endl;
                
                Vector3d lastC = trrPtsv1.back();
                double lastr = rListv1.back();
                double lastl = lListv1.back();
                bool lastw = wListv1.back();
                trrPtsv1.pop_back();
                rListv1.pop_back();
                lListv1.pop_back();
                wListv1.pop_back();
                
                double r = 0.5 * (preVec.norm()+rListv1.back()+rList[i]);
                Vector3d vectorbetween = trrSegPts[i+1] - lastC;
                Vector3d vbNorm = vectorbetween/vectorbetween.norm();
                Vector3d tempCenter = lastC + vbNorm * (r - rListv1.back());
                double tempR = min(r, min(longList[i], lListv1.back()));
                double tempL = max(r, min(longList[i], rListv1.back()));
                
                trrPtsv1.push_back(tempCenter);
                rListv1.push_back(tempR);
                lListv1.push_back(tempL);
                wListv1.push_back(widerList[i]); //undecided wider
                continue;
               
            }
            */
        }
        else{
            preVec = trrSegPts[i+1] - trrSegPts[i];
            Vector3d center = trrSegPts[i+1];
            Vector3d lb, ru;
            if(widerList[i] == 1){
                lb(0) = center(0) - longList[i]; lb(1) = center(1) - rList[i]; lb(2) = 0;
                ru(0) = center(0) + longList[i]; ru(1) = center(1) + rList[i]; ru(2) = 0;
            }else{
                lb(0) = center(0) - rList[i]; lb(1) = center(1) - longList[i]; lb(2) = 0;
                ru(0) = center(0) + rList[i]; ru(1) = center(1) + longList[i]; ru(2) = 0;
            }
            //findVertex(widerList[i], longList[i], rList[i],center, lb, ru);
            if(start_pt(0)<ru(0) && start_pt(0)>lb(0) && start_pt(1)<ru(1) && start_pt(1)>lb(1)){
                cout<<"Abandon first turning PT."<<endl;
                trrSegPts.erase(trrSegPts.begin()+1);
                rList.erase(rList.begin());
                longList.erase(longList.begin());
                widerList.erase(widerList.begin());
                curNum--;
                i--;
                continue;
            } 
                
        }
        trrPtsv1.push_back(trrSegPts[i+1]);
        rListv1.push_back(rList[i]);
        lListv1.push_back(longList[i]);
        wListv1.push_back(widerList[i]);
    }    
    cout<<"rListv1 size: "<<rListv1.size()<<endl;
    trrSegPts.clear();
    //trrSegPts.push_back(start_pt);
    for(auto pt : trrPtsv1)
        trrSegPts.push_back(pt);
    //trrSegPts.push_back(end_pt);
    cout<<"trrSeg size after considering distance: "<<trrSegPts.size()<<endl;

    vector<TurningNode> tempTurning;
    Vector3d pre_end, next_start;
    int curSize = trrSegPts.size();
    //while(1){
    for(int i = 0; i < trrSegPts.size(); i++){
        Vector3d thisturn = trrSegPts[i];
        double thisradius = rListv1[i];
        double thislong = lListv1[i];
        double thisw = wListv1[i];
        TurningNodePtr tempTurningPt = new TurningNode(thisturn, thisradius, thislong, thisw);
        TurningNode turnNode = *tempTurningPt;
        if(trrSegPts.size()==1){
            pre_end = start_pt;
            next_start = end_pt;
        }
        else if(i == 0){
            pre_end = start_pt;
            next_start = trrSegPts[i+1];
        }
        else if(i == trrSegPts.size()-1){
            if(!tempTurning.empty())
                pre_end = tempTurning.back().centerCoord;
            else
                pre_end = start_pt;
            next_start = end_pt;
        }
        else{
            if(!tempTurning.empty())
                pre_end = tempTurning.back().centerCoord;
            else
                pre_end = start_pt;
            next_start = trrSegPts[i+1];
        }
        
        if(lineCross(pre_end, next_start, turnNode) == 0){
            cout<<i<<"th turning doesn't crossed."<<endl;

            tempTurning.push_back(turnNode);  
        }            
        else{
            cout<<i<<"th turning crossed."<<endl;
            if(i!=trrSegPts.size()-1){
                i++;
                thisturn = trrSegPts[i];
                thisradius = rListv1[i];
                thislong = lListv1[i];
                thisw = wListv1[i];
                tempTurningPt = new TurningNode(thisturn, thisradius, thislong, thisw);
                turnNode = *tempTurningPt;
                tempTurning.push_back(turnNode);
            }
                //tempTurning.push_back(trrSegPts[++i]);
        }
            
    }
    //     if(curSize == tempTurning.size())
    //         break;
    //     else 
    //         curSize = tempTurning.size();
    // }
    trrSegPts.clear();
    turningPts.clear();
    rListv1.clear();
    lListv1.clear();
    wListv1.clear();

    trrSegPts.push_back(start_pt);
    for(auto pt : tempTurning){
        trrSegPts.push_back(pt.centerCoord);
        rListv1.push_back(pt.radius);
        lListv1.push_back(pt.longrad);
        wListv1.push_back(pt.wider);
    }
    trrSegPts.push_back(end_pt);
    cout<<"final key point number: "<< trrSegPts.size()<<endl;
    
        
        
    //Connect Final Remaining PTS
    for(int i = 0; i < rListv1.size(); i++){
        preVec = trrSegPts[i+1] - trrSegPts[i];
        preVecNorm = preVec/preVec.norm();
        nextVec = trrSegPts[i+2] - trrSegPts[i+1];
        nextVecNorm = nextVec/nextVec.norm();

        TurningNodePtr tempTurningPt = new TurningNode(trrSegPts[i+1], preVecNorm, nextVecNorm, rListv1[i], lListv1[i], wListv1[i]);
        TurningNode thisturn = *tempTurningPt;

        turningPts.push_back(thisturn);
    }   
    cout<<"number of turningNode back to main: "<< turningPts.size()<<endl;
}
/* 
void simpleTrajOptimizer::findVertex(double wider, double l, double r, Vector3d center, Vector3d& lb, Vector3d& ru){
    if(wider == 1){
        lb(0) = center(0) - l; lb(1) = center(1) - r; lb(2) = 0;
        ru(0) = center(0) + l; ru(1) = center(1) + r; ru(2) = 0;
    }else{
        lb(0) = center(0) - r; lb(1) = center(1) - l; lb(2) = 0;
        ru(0) = center(0) + r; ru(1) = center(1) + l; ru(2) = 0;
    }
}
*/
int simpleTrajOptimizer::lineCross(Vector3d pt1, Vector3d pt2, TurningNode turn){
    Vector3d center, lb, ru;
    center = turn.centerCoord;
    double k2 = (pt2(1)-center(1))/(pt2(0)-center(0));
    double k1 = (pt1(1)-center(1))/(pt1(0)-center(0));
    if(k1==k2){
        cout<<"Pass center."<<endl;
        return 1;
    }
    //cout<<"wider:"<<turn.wider<<endl;
    if(turn.wider == 1){
        lb(0) = center(0) - turn.longrad; lb(1) = center(1) - turn.radius; lb(2) = 0;
        ru(0) = center(0) + turn.longrad; ru(1) = center(1) + turn.radius; ru(2) = 0;
    }else{
        lb(0) = center(0) - turn.radius; lb(1) = center(1) - turn.longrad; lb(2) = 0;
        ru(0) = center(0) + turn.radius; ru(1) = center(1) + turn.longrad; ru(2) = 0;
    }

    double x1,y1,x2,y2;
    Vector3d t;
	if (pt1(1) > pt2(1)){
		t = pt1;
		pt1 = pt2;
		pt2 = t;
	}
    x1 = pt1(0); x2 = pt2(0); y1 = pt1(1); y2 = pt2(1);
    if (y1 == y2)	// 线段平行于x轴
	{
		if (y1 <= ru(1) && y1 >= lb(1)){
            int answer = IntervalOverlap(lb(0), ru(0), x1, x2);
            return answer;
        }
		else
			return 0;
	}
	double k = (x2 - x1)/(y2 - y1);
	Vector3d C, D;
	if (y1 < lb(1)){
		C(1) = lb(1);
		C(0) = k * (C(1) - y1) + x1;
	}
	else
		C = pt1;
		
	if (y2 > ru(1)){
		D(1) = ru(1);
		D(0) = k * (D(1) - y1) + x1;
	}
	else
		D = pt2;
		
	if (D(1) >= C(1))	// y维上有交集
		return IntervalOverlap(lb(0), ru(0), D(0), C(0));
	else
		return 0;
	
}

int IntervalOverlap(double x1, double x2, double x3, double x4)
{
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
}

