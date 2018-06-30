#include <emc/io.h>
#include <iostream>
#include "feature.h"
#include "config.h"
#include <fstream>
#include <cmath>

bool aquireAveragedData(int scans, Feature::AveragedLaserData &data, emc::IO *io, emc::Rate *r) {
    if (scans <= 0)
        return false;

    emc::LaserData rawScan;
    emc::OdometryData odom;

    memset(data.range, 0, sizeof(data.range));
    memset(data.succesfulScans, 0, sizeof(data.succesfulScans));
    while (scans-- > 0) {
        while (io->ok()) {
            if (io->readLaserData(rawScan)) {
                for (int i = 0; i < SCAN_POINTS; i++) {
                    if (rawScan.ranges[(i + SCAN_OFFSET) * SCAN_STEPSIZE] <
                            rawScan.range_min ||
                            rawScan.ranges[(i + SCAN_OFFSET) * SCAN_STEPSIZE] >
                            rawScan.range_max)
                        continue;
                    data.succesfulScans[i]++;
                    data.range[i] += double(rawScan.ranges[(i + SCAN_OFFSET) * SCAN_STEPSIZE]);
                    data.angle[i] += double(rawScan.angle_min) +
                            (i + SCAN_OFFSET) * SCAN_STEPSIZE *
                            double(rawScan.angle_increment);
                }
                break;
            }
            r->sleep();
        }
    }

    for (int i = 0; i < SCAN_POINTS; i++) {
        data.angle[i] /= data.succesfulScans[i];
        data.range[i] /= data.succesfulScans[i];
    }

    while(io->ok()){
        if(io->readOdometryData(odom)){
            data.x = odom.x;
            data.y = odom.y;
            data.theta = odom.a;
            break;
        }
        r->sleep();
    }
    return true;
}


//****************************** Preprocess *********************************
Feature::AveragedLaserData Feature::Preprocess(Feature::AveragedLaserData &laser) {
    Feature::AveragedLaserData ProcessedData;
    ProcessedData.vangle.clear();
    ProcessedData.vrange.clear();
    ProcessedData.vx.clear();
    ProcessedData.vy.clear();
    std::cout<<"Eneter Preprocess function !\n";
    for (int i=0;i<SCAN_POINTS;i++){
        if(std::abs(laser.angle[i])<=PI/2 && std::isnan(laser.angle[i])==false)
        {
            ProcessedData.theta = laser.theta;
            ProcessedData.vrange.push_back(laser.range[i]);
            ProcessedData.vangle.push_back(laser.angle[i]+laser.theta);  //wrt odom theta
            ProcessedData.vx.push_back(cos(laser.angle[i]+laser.theta)*laser.range[i]+laser.x); // wrt odom xy
            ProcessedData.vy.push_back(sin(laser.angle[i]+laser.theta)*laser.range[i]+laser.y);
        }
    }
    ProcessedData.x = laser.x;
    ProcessedData.y = laser.y;
    std::cout<<"Preprocess function works fine!\n";
    return ProcessedData;
}


// ******************************* Merge Scan************************************
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
    section_info.init.clear();
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
            if (section_info.end[tempindex]-section_info.init[tempindex]<FEATURE_SECT_OUTLIER){ // less than threshold points could be seen as outliers
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


// *************************************** get line parameter a b c *****************************************
Feature::linePar Feature::getLinePar(double x1, double y1, double x2, double y2){
    //    std::cout<<"Enter getLinePar function.\n";
    Feature::linePar linePar;
    linePar.a = (y1-y2)/(x1-x2);
    linePar.b = -1;
    linePar.c = (x1*y2-x2*y1)/(x1-x2);
    linePar.aa = linePar.a*linePar.a;
    linePar.bb = linePar.b*linePar.b;
    //    std::cout<<"getLinePar function works fine!\n";
    return linePar;
}


// *********************************** point to line dist **************************************************
double Feature::pointLineDist(double x, double y, Feature::linePar linePar){
    //    std::cout<<"Enter pointLinedist function.\n";
    double dis = std::abs(linePar.a*x + linePar.b*y + linePar.c)/std::sqrt(linePar.aa+linePar.bb);
    //    std::cout<<"pointLineDist function works fine!\n";
    return dis;
}


// ************************************* Feature Detection, return feature index in each section *****************************
std::vector<std::vector<double> > Feature::FeatureDetection(Feature::AveragedLaserData &laser, Feature::section_index &section_info){
    std::cout<<"Enter FeatureDetection function\n";
    std::vector<double> cornerindex;
    std::vector<std::vector<double> > cornermatrix;
    bool CornerFlag = true; // make program run, initialize with true
    for (int i=0;i<section_info.num;i++){ // loop in each section
        cornerindex.clear();
        cornerindex.push_back(section_info.init[i]); // push the init index to corner index
        while(CornerFlag){
            Feature::linePar linePar;
            double init_index = section_info.temp[i];
            double end_index  = section_info.end[i];
            linePar = Feature::getLinePar(laser.vx[init_index],laser.vy[init_index],laser.vx[end_index],laser.vy[end_index]);
            double maxdis = 0;   // initialize the max distance of point to the fitted line in current section
            double CornerIndex = section_info.temp[i];  // initialize corner index with section_info.temp in current section

            for (int k=section_info.temp[i]+1; k<section_info.end[i]-1; k++){ // loop within current section to find max distance point
                double dis = Feature::pointLineDist(laser.vx[k],laser.vy[k],linePar);
                if (dis>maxdis){  // update max distance point
                    maxdis = dis;
                    CornerIndex = k;
                }
            }
            if(maxdis > FEATURE_WALL){
                cornerindex.push_back(CornerIndex);// push info to struct corner
                std::cout<<"Corner found!\n";
                section_info.temp[i] = CornerIndex; // update section_info.temp with new corner
            }
            else{
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

// ******************************************** build corner struct from corner index matrix ***************************************************
std::vector<std::vector<Feature::CornerInfo> > Feature::index2Struct(std::vector<std::vector<double> > cornermatrix, Feature::AveragedLaserData &laser){
    std::cout<<"Enter index2Corner function.\n";
    std::vector<std::vector<Feature::CornerInfo> > cornerStruct;
    Feature::CornerInfo tempCorner;
    Feature::CornerInfo errorFlag;
    std::vector<Feature::CornerInfo> row;
    double sectNum = cornermatrix.size();
    errorFlag.x = 100;
    errorFlag.y = 100;
    errorFlag.flag = 100;
    std::cout<<"Section number: "<<sectNum<<std::endl;
    for(int i=0;i<sectNum;i++){
        std::cout<<"Section size: "<<cornermatrix[i].size()<<std::endl;
        for(int j=0;j<cornermatrix[i].size();j++){
            if((i==0 && j==0)||(i==sectNum-1 && j==cornermatrix[sectNum-1].size()-1)){ // the flag of initial and end point is different from others
                tempCorner.flag = 0;
            }
            else if(cornermatrix[i].size()>2){
                double corIndex  = cornermatrix[i][j];
                double preIndex  = corIndex - FEATURE_COMPARE;
                double postIndex = corIndex + FEATURE_COMPARE;
                double preDist   = Feature::pointsDist(laser.vx[preIndex],laser.vy[preIndex],laser.x,laser.y);
                double postDist  = Feature::pointsDist(laser.vx[postIndex],laser.vy[postIndex],laser.x,laser.y);
                double corDist   = Feature::pointsDist(laser.vx[corIndex],laser.vy[corIndex],laser.x,laser.y);
                if (preDist<corDist && postDist<corDist){ // corner of room
                    tempCorner.flag = 1;
                    std::cout<<"Find a corner of room!\n";
                }else if (preDist>corDist && postDist>corDist){ // corner of door
                    tempCorner.flag = 2;
                    std::cout<<"Find a corner of door!\n";
                }
            }
            else tempCorner.flag = -1; // unknown
            if(i!=sectNum-1 && j==cornermatrix[i].size()-1) tempCorner.flag = -1; // every end point except last section should be -1

            tempCorner.x = laser.vx[cornermatrix[i][j]];
            tempCorner.y = laser.vy[cornermatrix[i][j]];
            row.push_back(tempCorner);
        }
        row.push_back(errorFlag);
        cornerStruct.push_back(row);
        row.clear();
    }
    std::cout<<"index2Struct function works fine!\n";
    return cornerStruct;
    std::cout<<"index2Corner function works fine!\n";
}


// ******************************************** Merge Map of multiple index2struct ***********************************************************
bool Feature::MergeMap(std::vector<std::vector<Feature::CornerInfo> > &cornerStruct, std::vector<std::vector<Feature::CornerInfo> > &Map){
    std::cout<<"Enter MergeMap\n";
    for (int i = 0; i<cornerStruct.size();i++){
        Map.push_back(cornerStruct[i]);
    }
    std::cout<<"MergeMap function works fine!\n";
}


// ******************************************** Merge Map of different scan, save points coord directly ****************************************
bool Feature::MergeMap(std::vector<std::vector<double> > &Mapxy, std::vector<std::vector<double> > &cornermatrix, Feature::AveragedLaserData &laser){
    std::cout<<"Enter MergeMap function.\n";
    std::vector<double> empty;
    empty.clear();
    for (double i=0;i<cornermatrix.size();i++){ //loop in each section
        double MapSize = Mapxy.size(); // update Mapxy size
        Mapxy.push_back(empty);
        Mapxy.push_back(empty);
        for (double j=0;j<cornermatrix[i].size();j++){
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
bool Feature::saveMap(const char *file, const std::vector<std::vector<double> > &cornerStruct) {
    std::ofstream filestream(file);
    if (filestream.is_open()) {
        for (double i=0;i<cornerStruct.size();i++){
            for (double j=0;j<cornerStruct[i].size();j++){
                filestream <<cornerStruct[i][j]<<","; // csv, 1row x, 2row y, 3row x, 4row y ...
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

// ******************************************* save map as csv ****************************************************
bool Feature::saveMap(const char *file, const std::vector<std::vector<Feature::CornerInfo> > cornerStruct){
    std::ofstream filestream(file);
    if (filestream.is_open()){
        for (int i=0; i<cornerStruct.size();i++){
            for (int j=0; j<cornerStruct[i].size(); j++){
                filestream<<cornerStruct[i][j].x<<",";
            }
            filestream<<"\n";
            for (int j=0; j<cornerStruct[i].size(); j++){
                filestream<<cornerStruct[i][j].y<<",";
            }
            filestream<<"\n";
            for (int j=0; j<cornerStruct[i].size(); j++){
                filestream<<cornerStruct[i][j].flag<<",";
            }
            filestream<<"\n";
        }
        filestream.close();
        return true;
    }
    else{
        std::cout<<"Could not write to file!\n";
        return false;
    }
}

// **************************************** find door ******************************************************
bool Feature::findDoor(std::string mode, std::vector<std::vector<Feature::CornerInfo> > &cornerStruct){
    std::cout<<"Enter findDoor function.\n";
    std::vector<Feature::CornerInfo> door;
    Feature::CornerInfo tempDoorCorner;
    door.clear();
    int sectNum = cornerStruct.size();
    if (mode =="Corridor"){
        std::cout<<"Start find door in cooridor\n";
        //        Feature::linePar corridorRwall = Feature::getLinePar(cornermatrix[0][0].x,cornermatrix[0][0].y,cornermatrix[0][1].x,cornermatrix[0][1].y);
        //        double tempLen = cornermatrix[sectNum-1].size();
        //        Feature::linePar corridorLwall = Feature::getLinePar(cornermatrix[sectNum-1].back().x,cornermatrix[sectNum-1].back().y,cornermatrix[sectNum-1][tempLen-2].x,cornermatrix[sectNum-1][tempLen-2].y);
        for (int i=0; i<sectNum-1; i++){
            double tempLastIndex = cornerStruct[i].size()-2;
            double tempdist = Feature::pointsDist(cornerStruct[i][tempLastIndex],cornerStruct[i+1][0]);
            std::cout<<"Every gap dist "<<tempdist<<std::endl;
            if (tempdist>FEATURE_DOOR_MIN && tempdist<FEATURE_DOOR_MAX){
                std::cout<<"Satisfied door corner find!\n";
                tempDoorCorner.flag = 2;
                tempDoorCorner.x = cornerStruct[i].back().x; // last point of pre section
                tempDoorCorner.y = cornerStruct[i].back().y;
                door.push_back(tempDoorCorner);
                tempDoorCorner.x = cornerStruct[i+1][0].x; // first point of current section
                tempDoorCorner.y = cornerStruct[i+1][0].y;
                door.push_back(tempDoorCorner);
            }
        }
        std::printf("Find %d door in corridor.\n",int(door.size()/2));
    }
    else if (mode == "Room"){
        std::cout<<"Start find door inside room\n";
    }
    std::cout<<"findDoor function works fine!\n";
    return true;
}

// ****************************************** calculate 2 points distance ***********************************
double Feature::pointsDist(Feature::CornerInfo point1, Feature::CornerInfo point2){
    std::cout<<"Enter pintsDist function.\n";
    double dist;
    dist = std::sqrt(std::pow(point1.x-point2.x,2)+std::pow(point1.y-point2.y,2));
    std::cout<<point1.x<<","<<point1.y<<","<<point2.x<<","<<point2.y<<std::endl;
    //    std::cout<<"pointsDist function works fine!\n";
    return dist;
}

double Feature::pointsDist(double x1, double y1, double x2, double y2){
    //    std::cout<<"Enter pintsDist function.\n";
    double dist;
    dist = std::sqrt(std::pow(x1-x2,2)+std::pow(y1-y2,2));
    //    std::cout<<"pointsDist function works fine!\n";
    return dist;
}
