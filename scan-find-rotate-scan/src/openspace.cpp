// Instruction:
// In main function, let PICO stand still, scan once and save the LaserData as flaser
// Then rotate 180 deg, scan and save the LaserData as blaser.
// Call Openspace::MergeData(), the Openspace::Preprocess(*laser) in called inside.
// Then call Openspace::FindGap(),it will return the index of gap points.
// Call OpenspaceCoord(openindex), it will print the x-y coordinates of openspace.

#include <emc/io.h>
#include "openspace.h"
#include <vector>
#include <cmath>
#include <iostream>

// preprocess data:
// input
// 1. remove overlapping part
// 2. remove outlier, either extreme angle, or NaN type
// 3. get absolute angle wrt odom theta
// output a new struct, with range, dis // odom's x, y, theta
Openspace::AveragedLaserData Openspace::Preprocess(Openspace::AveragedLaserData &laser) { //check input! check function type!

    Openspace::AveragedLaserData ProcessedData;
    std::cout<<"Eneter Preprocess function"<<std::endl;
    for (int i=0;i<SCAN_POINTS;i++){
        if(std::abs(laser.angle[i])<PI/2 && std::isnan(laser.angle[i])==false)//check if nan happens  && isnan(laser.angle[i]==false)
        {
            ProcessedData.vrange.push_back(laser.range[i]);
            ProcessedData.vangle.push_back(laser.angle[i]-laser.theta);  //wrt odom theta      -laser.theta
        }
        else{
            //std::cout<<"Invalid data find"<<std::endl;
        }
    }
    std::cout<<"Preprocess function works fine!"<<std::endl;
    return ProcessedData;
}


// merge two scan (front and back) into single variable
Openspace::AveragedLaserData Openspace::MergeData(Openspace::AveragedLaserData &flaser,Openspace::AveragedLaserData &blaser){
    Openspace::AveragedLaserData tempf = Preprocess(flaser);
    Openspace::AveragedLaserData tempb = Preprocess(blaser);
    Openspace::AveragedLaserData Claser;

    Claser.vangle.insert(Claser.vangle.begin(),tempf.vangle.begin(),tempf.vangle.end());
    Claser.vrange.insert(Claser.vrange.begin(),tempf.vrange.begin(),tempf.vrange.end());

    Claser.vangle.insert(Claser.vangle.end(),tempb.vangle.begin(),tempb.vangle.end());// check each element inside struct claser's size
    Claser.vrange.insert(Claser.vrange.end(),tempb.vrange.begin(),tempb.vrange.end());
    std::cout<<"MergeData function works fine!"<<std::endl;
    return Claser;
}


// find the boundary points of any openspace
std::vector<double> Openspace::FindGap(Openspace::AveragedLaserData &Claser){
    std::vector<double> openindex;
    std::cout<<"Enter FindGap function"<<std::endl;
    double deltadis[Claser.vrange.size()];
    //std::cout<<"FindGap define deltadis"<<std::endl;
    std::cout<<"Laser size is:  "<<Claser.vrange.size()<<std::endl;

    // measure the gap distance with threshold
    for (int i=0;i<Claser.vrange.size()-1;i++){
        //std::cout<<"FindGap run once begin"<<std::endl;
        double temp1 = std::pow(Claser.vrange[i],2);
        double temp2 = std::pow(Claser.vrange[i+1],2);
        double temp3 = 2*Claser.vrange[i]*Claser.vrange[i+1]*cos(Claser.vangle[i]-Claser.vangle[i+1]);
        deltadis[i] = std::sqrt(temp1+temp2-temp3); // r1^2+r2^2-2*r1*r2*cos(delta_angle)
        if (deltadis[i]>std::pow(GAP,2)){
            openindex.push_back(i);
            openindex.push_back(i+1); // need add one more point to form a "door".
            std::cout<<"The raw gap distance that found is: "<<std::sqrt(deltadis[i])<<std::endl;
        }
    }

    std::cout<<"Now checking if index exceeds data range!"<<std::endl;
    std::cout<<"If openindex is empty: "<<openindex.empty()<<std::endl;
    std::cout<<"Openindex size is: "<<openindex.size()<<std::endl;
    //std::cout<<"The last value of openindex is: "<<openindex.back()<<std::endl;
    //std::cout<<"The laser data range is: "<<Claser.vangle.size()<<std::endl;
    if(openindex.empty()){ // openindex.back()>Claser.vangle.size()-1 ||
        std::cout<<"Not find the gap!"<<std::endl;
        //openindex.clear();   // The vector::clear() seems something wrong, may cause core dumped

        // insert -1 to first of vector as a false flag
        openindex.insert(openindex.begin(),-1);
        std::cout<<"Cleared openindex"<<std::endl;
    }

    else{
        // if the gap point is far away from threshold
        std::cout<<"Openindex not exceed data range"<<std::endl;
        for (int i=0;i<openindex.size();i++){
            if (Claser.vrange[i]>FAR){
                openindex.erase(openindex.begin()+i);
                std::cout<<"Remove the points that far from FAR THRESHOLD"<<std::endl;
            }
        }
        std::cout<<"Extreme far points are removed"<<std::endl;
        std::cout<<"Now the openindex size is: "<<openindex.size()<<std::endl;
    }
    return openindex;
}


// convert to coordinate, not considered odom xy now, because by default the robot is at (0,0)
Openspace::OpenCoord Openspace::OpenspaceCoord(std::vector<double> &openindex, Openspace::AveragedLaserData &Claser, double &robotx, double &roboty){
    Openspace::OpenCoord openxy;
    for (int i=0;i<openindex.size();i++){
        double temp = openindex[i];
        openxy.x.push_back(robotx+Claser.vrange[temp]*cos(Claser.vangle[temp]));// wrt odom!
        openxy.y.push_back(roboty+Claser.vrange[temp]*sin(Claser.vangle[temp]));// wrt odom! otherwise, remove odom.xy
    }
    return openxy;
}
