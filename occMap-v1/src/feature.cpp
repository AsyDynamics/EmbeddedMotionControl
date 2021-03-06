#include <emc/io.h>
#include <iostream>
#include "feature.h"
#include "config.h"
#include <fstream>
#include <cmath>


//****************************** Preprocess *********************************
Feature::AveragedLaserData Feature::Preprocess(Feature::AveragedLaserData &laser) { //check input! check function type!
    Feature::AveragedLaserData ProcessedData;

    ProcessedData.vangle.clear();
    ProcessedData.vrange.clear();
    ProcessedData.vx.clear();
    ProcessedData.vy.clear();
    std::cout<<"Eneter Preprocess function !\n";
    std::cout<<"The odom theta in preprocess is: "<<laser.theta<<std::endl;
    for (int i=0;i<SCAN_POINTS;i++){
        if(std::abs(laser.angle[i])<=PI/2 && std::isnan(laser.angle[i])==false)//check if nan happens  && isnan(laser.angle[i]==false)
        {
            ProcessedData.theta = laser.theta;
            ProcessedData.vrange.push_back(laser.range[i]);
            ProcessedData.vangle.push_back(laser.angle[i]+laser.theta);  //wrt odom theta
            ProcessedData.vx.push_back(cos(laser.angle[i]+laser.theta)*laser.range[i]+laser.x); // wrt odom xy, absolute xy coord
            ProcessedData.vy.push_back(sin(laser.angle[i]+laser.theta)*laser.range[i]+laser.y);
        }
    }
    ProcessedData.x = laser.x;
    ProcessedData.y = laser.y; // no need of this
    std::cout<<"Processed Odom theta is: "<<ProcessedData.theta<<std::endl;
    std::cout<<"Processed Odom xy coord is: "<<ProcessedData.x<<','<<ProcessedData.y<<std::endl;
    std::cout<<"Preprocess function works fine!\n";
    return ProcessedData;
}


// ******************************* Merge Scan (optional) ************************************
Feature::AveragedLaserData Feature::MergeScan(Feature::AveragedLaserData &flaser, Feature::AveragedLaserData &blaser){
    std::cout<<"Eneter Merge function\n";
    Feature::AveragedLaserData tempf = Preprocess(flaser);
    Feature::AveragedLaserData tempb = Preprocess(blaser);
    Feature::AveragedLaserData Claser;

    Claser.vangle.insert(Claser.vangle.begin(),tempf.vangle.begin(),tempf.vangle.end());
    Claser.vrange.insert(Claser.vrange.begin(),tempf.vrange.begin(),tempf.vrange.end());

    Claser.vangle.insert(Claser.vangle.end(),tempb.vangle.begin(),tempb.vangle.end());// check each element inside struct claser's size
    Claser.vrange.insert(Claser.vrange.end(),tempb.vrange.begin(),tempb.vrange.end());

    Claser.vx.insert(Claser.vx.begin(),tempf.vx.begin(),tempf.vx.end());
    Claser.vy.insert(Claser.vy.begin(),tempf.vy.begin(),tempf.vy.end());

    Claser.vx.insert(Claser.vx.end(),tempb.vx.begin(),tempb.vx.end());
    Claser.vy.insert(Claser.vy.end(),tempb.vy.begin(),tempb.vy.end());
    std::cout<<"Merge function works fine!\n";
    return Claser;
}

// ******************************* FindSection ***********************************
Feature::section_index Feature::FindSection(Feature::AveragedLaserData &laser){
    std::cout<<"Enter FindSection funtion\n";
    Feature::section_index section_info;
    section_info.init.clear(); // very important !!!!! otherwise cause random value
    section_info.end.clear();;
    section_info.temp.clear();
    double deltadis[laser.vrange.size()];
    section_info.init.push_back(0); // The first section's inital index
    section_info.temp.push_back(0); // initialize temp as init, for later use
    section_info.num = 1;

    for (int i=0;i<laser.vrange.size()-1;i++){
        double temp1 = std::pow(laser.vx[i]-laser.vx[i+1],2);
        double temp2 = std::pow(laser.vy[i]-laser.vy[i+1],2);
        deltadis[i]  = temp1+temp2;
        if (deltadis[i]>std::pow(FEATURE_GAP,2)){
            section_info.end.push_back(i);   //it's first point of a gap, also end point of section
            section_info.num++;
            double tempindex = section_info.num-2;
            if (section_info.end[tempindex]-section_info.init[tempindex]<FEATURE_OUTLIER){ // less than threshold points could be seen as outliers
                std::cout<<"The current section size: "<<section_info.end[tempindex]-section_info.init[tempindex]<<std::endl;
                section_info.init.erase(section_info.init.begin()+tempindex); // remove this set of index
                section_info.end.erase(section_info.end.begin()+tempindex);
                section_info.temp.erase(section_info.temp.begin()+tempindex);
                section_info.num--;
            }
            section_info.init.push_back(i+1);  //end point of a gap, initial point of section
            section_info.temp.push_back(i+1);
        }
    }
    section_info.end.push_back(laser.vangle.size()-1); // The last secton's end index
    section_info.num = section_info.init.size();
    section_info.temp.push_back(section_info.init[section_info.num-1]); // in for loop, it only push n-1 time

    std::cout<<"FindSection function works fine!\n";
    std::cout<<"Find sections in total: "<<section_info.num<<std::endl;
    return section_info;
}

// ************************************* Feature Detection, return feature index in each section ***************************************
std::vector<std::vector<double> > Feature::FeatureDetection(Feature::AveragedLaserData &laser, Feature::section_index &section_info){
    std::cout<<"Enter FeatureDetection function\n";
    //Feature::CornerInfo corner;
    std::vector<double> cornerindex;
    std::vector<std::vector<double> > cornermatrix;
    cornermatrix.clear();
    bool CornerFlag = true; // make program run, initialize with true
    //std::cout<<"Defined cornerflag for while loop, type bool\nSection num in total:  "<<section_info.num<<std::endl;
    for (int i=0;i<section_info.num;i++){ // loop in each section
        cornerindex.clear();
        cornerindex.push_back(section_info.init[i]); // push the init index to corner index
        while(CornerFlag){
            double init_index = section_info.temp[i];
            double end_index  = section_info.end[i];

            double x1 = laser.vx[init_index];
            double y1 = laser.vy[init_index];
            double x2 = laser.vx[end_index];
            double y2 = laser.vy[end_index];

            double a = (y1-y2)/(x1-x2);
            double b = -1;
            double c = (x1*y2-x2*y1)/(x1-x2);
            std::cout<<"The line coefficients a,b,c are: "<<a<<","<<b<<","<<c<<std::endl;

            double maxdis = 0;   // initialize the max distance of point to the fitted line in current section
            double CornerIndex = section_info.temp[i];  // initialize corner index with section_info.temp in current section

            for (int k=section_info.temp[i]+1; k<section_info.end[i]-1; k++){ // loop within current section to find max distance point
                double dis = std::abs(a*laser.vx[k]+b*laser.vy[k]+c)/std::sqrt(a*a+b*b);
                if (dis>maxdis){  // update max distance point
                    maxdis = dis;
                    CornerIndex = k;
                }
            }
            std::cout<<"Maxdis point index: "<<CornerIndex<<", Max dis: "<<maxdis<<std::endl;

            if(maxdis > FEATURE_WALL){// Corner found!
                cornerindex.push_back(CornerIndex);// push info to struct corner
                std::cout<<"Corner found!\n";
                section_info.temp[i] = CornerIndex; // update section_info.temp with new corner
            }
            else{                      // It's wall
                std::cout<<"Wall found! \n";
                CornerFlag = false;
            }
        }
        CornerFlag = true;
        cornerindex.push_back(section_info.end[i]);
        cornermatrix.push_back(cornerindex);
    }
    std::cout<<"FeatureDetectoin works fine!\n";
    return cornermatrix;

}

// ******************************************** Merge Map of different scan, save points coord directly ****************************************
bool Feature::MergeMap(std::vector<std::vector<double> > &Mapxy, std::vector<std::vector<double> > &cornermatrix, Feature::AveragedLaserData &laser){
    std::cout<<"Enter MergeMap function.\n";
    std::vector<double> empty;
    empty.clear();
    for (double i=0;i<cornermatrix.size();i++){ //loop in each section
        //Map.push_back(cornermatrix[i]);
        double MapSize = Mapxy.size(); // update Mapxy size
        Mapxy.push_back(empty);
        Mapxy.push_back(empty);
        std::cout<<"In MergeMap function, in "<<i<<" section\n";
        for (double j=0;j<cornermatrix[i].size();j++){
            std::cout<<"In MergeMap function, merging "<<j<<" point\n";
            double temp = cornermatrix[i][j];
            Mapxy[MapSize].push_back(laser.vx[temp]);
            Mapxy[MapSize+1].push_back(laser.vy[temp]);
        }
        Mapxy[MapSize].push_back(100); // indicate error, only used for plot in matlab
        Mapxy[MapSize+1].push_back(100); // indicate error
    }
    std::cout<<"The total secion number of world is: "<<Mapxy.size()/2<<std::endl;
    std::cout<<"MergeMap function works fine!\n";
    return true;
}


// ******************************************** save map as csv ****************************************************
bool Feature::saveMap(const char *file, const std::vector<std::vector<double> > &Mapxy) {
    std::ofstream filestream(file);

    if (filestream.is_open()) {
        for (double i=0;i<Mapxy.size();i++){
            for (double j=0;j<Mapxy[i].size();j++){
                filestream <<Mapxy[i][j]<<","; // csv, 1row x, 2row y, 3row x, 4row y ...
            }
            filestream<< "\n";
        }
        filestream.close();
        return true;
    } else {
        std::cout << "Could not write to file!\n";
        return false;
    }
}

// ********************************************** update occupancy map *********************************************
bool Feature::updateOccupancyMap(double OccMap[FEATURE_MAP_X/FEATURE_MAP_R][FEATURE_MAP_Y/FEATURE_MAP_R], Feature::section_index &sect_info, Feature::AveragedLaserData &Claser){
    // step one, write the ceil with obstacle, use ceil and floor
    for (int i=0; i<sect_info.num; i++){
        std::cout<<"Updating "<<i<<" section data\n";
        for (int j=sect_info.init[i]; j<=sect_info.end[i]; j++){
            int x, y;
            if (Claser.vx[j]>0){
                x = FEATURE_MAP_X/FEATURE_MAP_R/2 - std::ceil(Claser.vx[j]*100/FEATURE_MAP_R);// - FEATURE_MAP_PARKING_X/FEATURE_MAP_R;
            }
            else{
                x = FEATURE_MAP_X/FEATURE_MAP_R/2 - std::floor(Claser.vx[j]*100/FEATURE_MAP_R);// - FEATURE_MAP_PARKING_X/FEATURE_MAP_R;
            }
            if(Claser.vy[j]>0){
                y = - std::ceil(Claser.vy[j]*100/FEATURE_MAP_R)+FEATURE_MAP_Y/FEATURE_MAP_R/2; // start from half of the matrix
            }
            else{
                y = - std::floor(Claser.vy[j]*100/FEATURE_MAP_R)+FEATURE_MAP_Y/FEATURE_MAP_R/2;
            }
            OccMap[x][y] = -1;
            std::cout<<"Updating "<<j<<" point data\n";
        }
    }
    // step two, find the free point set that between odom and each laser beam
    // step three, write the free ceil
    return true;
}

// ***************************************** save occupancy map as csv *************************************************
bool Feature::saveOccupancyMap(const char *file, const double OccMap[FEATURE_MAP_X/FEATURE_MAP_R][FEATURE_MAP_Y/FEATURE_MAP_R]){
    std::ofstream filestream(file);

    if (filestream.is_open()) {
        for (int i=0; i<FEATURE_MAP_X/FEATURE_MAP_R; i++){
            for (int j=0; j<FEATURE_MAP_Y/FEATURE_MAP_R; j++){
                filestream <<OccMap[i][j]<<","; // csv, occupancy map
            }
            filestream<< "\n";
        }
        filestream.close();
        return true;
    } else {
        std::cout << "Could not write to file!\n";
        return false;
    }
}
