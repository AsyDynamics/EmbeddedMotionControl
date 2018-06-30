#include <emc/io.h>
#include <iostream>
//#include "monetary.h"
#include "config.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include "monitoring.h"

using namespace std;

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
Feature::AveragedLaserData Feature::Preprocess(Feature::AveragedLaserData &laser, Feature::bias Bias) {
    cout<<"Eneter Preprocess function !\n";
    Feature::AveragedLaserData ProcessedData;
    ProcessedData.vangle.clear();
    ProcessedData.vrange.clear();
    ProcessedData.vx.clear();
    ProcessedData.vy.clear();
    for (int i=0;i<SCAN_POINTS;i++){
        if(abs(laser.angle[i])<=PI/2 && isnan(laser.angle[i])==false)
        {
            ProcessedData.theta = laser.theta;
            ProcessedData.vrange.push_back(laser.range[i]);
            ProcessedData.vangle.push_back(laser.angle[i]+laser.theta+Bias.theta);  //wrt odom theta
            ProcessedData.vx.push_back(cos(laser.angle[i]+laser.theta)*laser.range[i]+laser.x+Bias.x); // wrt odom xy
            ProcessedData.vy.push_back(sin(laser.angle[i]+laser.theta)*laser.range[i]+laser.y+Bias.y);
        }
    }
    ProcessedData.x = laser.x;
    ProcessedData.y = laser.y;
    cout<<"Preprocess function works fine!\n";
    return ProcessedData;
}


// ******************************* Merge Scan************************************
Feature::AveragedLaserData Feature::MergeScan(Feature::AveragedLaserData &flaser, Feature::AveragedLaserData &blaser,Feature::bias bias){
    cout<<"Eneter Merge function\n";
    Feature::AveragedLaserData tempf = Preprocess(flaser,bias);
    Feature::AveragedLaserData tempb = Preprocess(blaser,bias);
    Feature::AveragedLaserData Claser;

    Claser.vangle.insert(Claser.vangle.begin(),tempf.vangle.begin(),tempf.vangle.end());
    Claser.vrange.insert(Claser.vrange.begin(),tempf.vrange.begin(),tempf.vrange.end());

    Claser.vangle.insert(Claser.vangle.end(),tempb.vangle.begin(),tempb.vangle.end());// check each element inside struct claser's size
    Claser.vrange.insert(Claser.vrange.end(),tempb.vrange.begin(),tempb.vrange.end());

    Claser.vx.insert(Claser.vx.begin(),tempf.vx.begin(),tempf.vx.end());
    Claser.vy.insert(Claser.vy.begin(),tempf.vy.begin(),tempf.vy.end());

    Claser.vx.insert(Claser.vx.end(),tempb.vx.begin(),tempb.vx.end());
    Claser.vy.insert(Claser.vy.end(),tempb.vy.begin(),tempb.vy.end());
    cout<<"Merge function works fine!\n";
    return Claser;
}


// ******************************* FindSection ***********************************
Feature::section_index Feature::FindSection(Feature::AveragedLaserData &laser){
    cout<<"Enter FindSection funtion\n";
    Feature::section_index section_info;
    section_info.init.clear();
    section_info.end.clear();;
    section_info.temp.clear();
    double deltadis[laser.vrange.size()];
    section_info.init.push_back(0); // The first section's inital index
    section_info.temp.push_back(0); // initialize temp as init, for later use
    section_info.num = 1;

    for (int i=0;i<laser.vrange.size()-1;i++){
        double temp1 = pow(laser.vx[i]-laser.vx[i+1],2);
        double temp2 = pow(laser.vy[i]-laser.vy[i+1],2);
        deltadis[i]  = temp1+temp2;
        if (deltadis[i]>pow(FEATURE_GAP,2)){
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
    cout<<"FindSection function works fine!\n";
    return section_info;
}

// ************************************* Feature Detection, return feature index in each section *****************************
vector<vector<double> > Feature::FeatureDetection(Feature::AveragedLaserData &laser, Feature::section_index &section_info){
    cout<<"Enter FeatureDetection function\n";
    vector<double> cornerindex;
    vector<vector<double> > cornermatrix;
    Feature::linePar linePar;
    bool CornerFlag = true;                  // make program run, initialize with true
    for (int i=0;i<section_info.num;i++){    // loop in each section
        cornerindex.clear();
        cornerindex.push_back(section_info.init[i]); // push the init index to corner index
        while(CornerFlag){
            double init_index = section_info.temp[i];
            double end_index  = section_info.end[i];
            linePar = getLinePar(laser.vx[init_index],laser.vy[init_index],laser.vx[end_index],laser.vy[end_index]);
            double maxdis = 0;               // initialize the max distance of point to the fitted line in current section
            double CornerIndex = section_info.temp[i];  // initialize corner index with section_info.temp in current section

            for (int k = section_info.temp[i]+1; k<section_info.end[i]-1; k++){   // loop within current section to find max distance point
                double dis = pointLineDist(laser.vx[k],laser.vy[k],linePar);
                if (dis>maxdis){             // update max distance point
                    maxdis = dis;
                    CornerIndex = k;
                }
            }
            if(maxdis > FEATURE_WALL){
                cornerindex.push_back(CornerIndex);  // push info to struct corner
                cout<<"Corner found!\n";
                section_info.temp[i] = CornerIndex;  // update section_info.temp with new corner
            }
            else{
                cout<<"Wall found! \n";
                CornerFlag = false;
            }
        }
        CornerFlag = true;
        cornerindex.push_back(section_info.end[i]);
        cornermatrix.push_back(cornerindex);
    }
    cout<<"FeatureDetectoin works fine!\n";
    return cornermatrix;
}

// ******************************************** Feature Detection, stupid version, overcome missing corner **********************************
vector<vector<double> > Feature::FeatureDetection2(Feature::AveragedLaserData &laser, Feature::section_index &section_info){
    cout<<"Enter FeatureDetection function\n";
    vector<double> cornerindex;
    vector<vector<double> > cornermatrix;
    Feature::linePar linePar1, linePar2, linePar3;
    int tempIndex1, tempIndex2, tempIndex3;
    for (int i = 0; i<section_info.num;i++){ // loop between section
        cornerindex.push_back(section_info.init[i]);
        int init_index = section_info.init[i];
        int end_index  = section_info.end[i];
        linePar1 = getLinePar(laser.vx[init_index],laser.vy[init_index],laser.vx[end_index],laser.vy[end_index]);
        double maxdis = 0;
        for (int k1 = section_info.init[i]+1; k1<section_info.end[i]-1;k1++){ // in each section
            double dis1 = pointLineDist(laser.vx[k1],laser.vy[k1],linePar1);
            if (dis1>maxdis){
                maxdis = dis1;
                tempIndex1 = k1;
            }
        }
        if (maxdis>FEATURE_WALL){
            cornerindex.push_back(tempIndex1);

            linePar2 = getLinePar(laser.vx[init_index],laser.vy[init_index],laser.vx[tempIndex1],laser.vy[tempIndex1]); //for safe, fit two lines
            linePar3 = getLinePar(laser.vx[end_index],laser.vy[end_index],laser.vx[tempIndex1],laser.vy[tempIndex1]);
            maxdis = 0; // line2
            for (int k2 = section_info.init[i]+1; k2<tempIndex1-1; k2++){
                double dis2 = pointLineDist(laser.vx[k2], laser.vy[k2],linePar2);
                if(dis2>maxdis){
                    maxdis = dis2;
                    tempIndex2 = k2;
                }
            }
            if(maxdis>FEATURE_WALL){
                cornerindex.push_back(tempIndex2);
            }
            maxdis = 0; // line3
            for (int k3 = tempIndex1+1; k3<section_info.end[i]-1; k3++){
                double dis3 = pointLineDist(laser.vx[k3], laser.vy[k3], linePar3);
                if(dis3>maxdis){
                    maxdis = dis3;
                    tempIndex3 = k3;
                }
            }
            if(maxdis>FEATURE_WALL){
                cornerindex.push_back(tempIndex3);
            }
        }
        cornerindex.push_back(section_info.end[i]);
        sort(cornerindex.begin(),cornerindex.end());
        cornermatrix.push_back(cornerindex);
        cornerindex.clear(); // otherwise it push some history data
    }
    cout<<"FeatureDetectoin works fine!\n";
    return cornermatrix;
}

// ******************************************** build corner struct from corner index matrix ***************************************************
vector<vector<Feature::CornerInfo> > Feature::index2Struct(std::string mode, vector<vector<double> > cornermatrix, Feature::AveragedLaserData &laser, double y1, double y2){
    cout<<"Enter index2Struct function.\n";
    vector<vector<Feature::CornerInfo> > cornerStruct;
    Feature::CornerInfo tempCorner;
    vector<Feature::CornerInfo> row;

    cout<<"Start convert from index to struct.\n";
    for(int i=0;i<cornermatrix.size();i++){
        cout<<"This is "<<i+1<<" section\n";
        for(int j=0;j<cornermatrix[i].size();j++){
            if(j==0 || j==cornermatrix[i].size()-1){ // the flag of initial and end point is different from others
                tempCorner.flag = -1;
            }
            else {
                double corIndex  = cornermatrix[i][j];
                double preIndex  = corIndex - FEATURE_COMPARE;
                double postIndex = corIndex + FEATURE_COMPARE;
                double preDist   = Feature::pointsDist(laser.vx[preIndex],laser.vy[preIndex],laser.x,laser.y);
                double postDist  = Feature::pointsDist(laser.vx[postIndex],laser.vy[postIndex],laser.x,laser.y);
                double corDist   = Feature::pointsDist(laser.vx[corIndex],laser.vy[corIndex],laser.x,laser.y);
                if (preDist<corDist && postDist<corDist){       // corner of room
                    tempCorner.flag = 1;
                    cout<<"Find a corner of room!\n";
                }else if (preDist>corDist && postDist>corDist){ // corner of door
                    tempCorner.flag = 2;
                    cout<<"Find a corner of door!\n";
                }
                else tempCorner.flag = -1;
            }
            tempCorner.x = laser.vx[cornermatrix[i][j]];
            tempCorner.y = laser.vy[cornermatrix[i][j]];
            row.push_back(tempCorner);
        }
        cornerStruct.push_back(row);
        row.clear();
    }
    row.clear(); // later used to store door temp

    int flag = (mode == "Corridor" ?1:2);
    cout<<"Start remove no interest section. ";
    switch (flag){
    case 1:
        cout<<"Mode Corridor\n";
        for (int i=0;i<cornerStruct.size();i++){
            if (cornerStruct[i][0].y>0 && cornerStruct[i].back().y>0){ // both end of section is y>0
                if (cornerStruct[i][0].y-y2>FEATURE_BOUND_THRESHOLD && cornerStruct[i].back().y-y2>FEATURE_BOUND_THRESHOLD){
                    cout<<"Find section not belong to corridor!\n";
                    cornerStruct.erase(cornerStruct.begin()+i);
                    i--;
                }
            }else if(cornerStruct[i][0].y<0 && cornerStruct[i].back().y<0){ // both end of section is y<0
                if (-cornerStruct[i][0].y+y1>FEATURE_BOUND_THRESHOLD && -cornerStruct[i].back().y+y1>FEATURE_BOUND_THRESHOLD){ // reverse
                    cout<<"Find section not belong to corridor!\n";
                    cornerStruct.erase(cornerStruct.begin()+i);
                    i--;
                }
            }
        }
        break;
    case 2:
        cout<<"Mode Room\n";
        for (int i=0;i<cornerStruct.size();i++){
            for(int j=0;j<cornerStruct[i].size();j++){
                if (cornerStruct[i][j].flag==2){
                    tempCorner.x = cornerStruct[i][j].x;
                    tempCorner.y = cornerStruct[i][j].y;
                    tempCorner.flag = i;  // this flag is not the same as before, to store section number used later
                    row.push_back(tempCorner);
                }
            }
        }
        switch(row.size()){
        case 2:
            cout<<"Find two exit points, start to remove section.\n";
            if (row[1].flag-row[0].flag>1){
                int eraseMarker = row[0].flag+1;
                for (int i=row[0].flag;i<row[1].flag-1;i++){
                    cornerStruct.erase(cornerStruct.begin()+eraseMarker);
                }
            }
            break;
        case 3:
            cout<<"Find three exit points, start to remove section.\n";
            int eraseMarker = row[0].flag+1;
            for (int i =row[0].flag;i<row[2].flag-1;i++){
                cornerStruct.erase(cornerStruct.begin()+eraseMarker);
            }
            break;
        }
        break;
    }
    cout<<"index2Struct function works fine!\n";
    return cornerStruct;
}


// ******************************************** Merge Map of multiple index2struct ***********************************************************
bool Feature::MergeMap(vector<vector<Feature::CornerInfo> > &cornerStruct, vector<vector<Feature::CornerInfo> > &Map){
    cout<<"Enter MergeMap\n";
    for (int i = 0; i<cornerStruct.size();i++){
        Map.push_back(cornerStruct[i]);
    }
    cout<<"MergeMap function works fine!\n";
}


// ******************************************** Merge Map of different scan, save points coord directly ****************************************
bool Feature::MergeMap(vector<vector<double> > &Mapxy, vector<vector<double> > &cornermatrix, Feature::AveragedLaserData &laser){
    cout<<"Enter MergeMap function.\n";
    vector<double> empty;
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
    cout<<"The total secion number of world is: "<<Mapxy.size()/2<<endl;
    cout<<"MergeMap function works fine!\n";
    return true;
}


// ******************************************** save map as csv ****************************************************
bool Feature::saveMap(const char *file, const vector<vector<double> > &cornerStruct) {
    ofstream filestream(file);
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
        cout << "Could not write to file!\n";
        return false;
    }
}

// ******************************************* save map as csv ****************************************************
bool Feature::saveMap(const char *file, const vector<vector<Feature::CornerInfo> > cornerStruct){
    ofstream filestream(file);
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
        cout<<"Could not write to file!\n";
        return false;
    }
}

// **************************************** find door ******************************************************
vector<Feature::CornerInfo> Feature::findDoor(string mode, vector<vector<Feature::CornerInfo> > &cornerStruct){
    cout<<"Enter findDoor function.\n";
    vector<Feature::CornerInfo> tempDoor;
    vector<Feature::CornerInfo> door;
    int tempSect;
    int num2Corner = 0;
    Feature::CornerInfo tempDoorCorner;
    tempDoor.clear();
    int sectNum = cornerStruct.size();
    if (mode =="Corridor"){
        cout<<"Start find door in cooridor\n";
        for (int i=0;i<sectNum;i++){
            cout<<"Start scan section "<<i<<endl;
            for (int j=0;j<cornerStruct[i].size();j++){
                if (cornerStruct[i][j].flag == 2){           // first check flag, if there is 2, better
                    cout<<"Find a exit corner\n";
                    num2Corner+=1;
                    tempDoorCorner.x = cornerStruct[i][j].x;
                    tempDoorCorner.y = cornerStruct[i][j].y;
                    tempDoor.push_back(tempDoorCorner);
                    tempSect = i;
                }
            }
        }
        switch(tempDoor.size()){
        case 2:
            cout<<"Case 2\n";
            double doorWidth;
            doorWidth = pointsDist(tempDoor[0],tempDoor[1]);
            if (doorWidth>FEATURE_DOOR_MIN && doorWidth<FEATURE_DOOR_MAX){
                cout<<"Find a door defined by two flag corner!\n";
                door.push_back(tempDoor[0]);
                door.push_back(tempDoor[1]);
            }
            break;
        case 1:
            cout<<"Case 1\n";
            double doorWidth1;
            cout<<tempSect<<endl;
            doorWidth1 = pointsDist(tempDoor[0],cornerStruct[tempSect-1].back()); // previous section's last point
            double doorWidth2;
            doorWidth2= pointsDist(tempDoor[0],cornerStruct[tempSect+1][0]);      // next section's initial point
            cout<<"Two trial width: "<<doorWidth1<<","<<doorWidth2<<" current section: "<<tempSect<<endl;
            if (doorWidth1>FEATURE_DOOR_MIN && doorWidth1<FEATURE_DOOR_MAX){
                if (tempSect>0){ // for safe of not exceeding index
                    if (abs(tempDoor[0].x-cornerStruct[tempSect-1].back().x)<FEATURE_DOOR_XY_THRESHOLD || abs(tempDoor[0].y-cornerStruct[tempSect-1].back().y)<FEATURE_DOOR_XY_THRESHOLD){
                        cout<<"Find a door defined by one flag corner. It's in "<<tempSect<<" and previous section\n";
                        door.push_back(tempDoor[0]);
                        door.push_back(cornerStruct[tempSect-1].back());
                        cornerStruct[tempSect-1].back().flag = 2; // reset flag
                    }
                }
            }
            if(doorWidth2>FEATURE_DOOR_MIN && doorWidth2<FEATURE_DOOR_MAX){ // cannot use else if, because sometimes these two satisfies at same time, may miss door
                if (tempSect+1<sectNum){ // for safe of not exceeding index
                    cout<<abs(tempDoor[0].x-cornerStruct[tempSect+1][0].x)<<","<<abs(tempDoor[0].y-cornerStruct[tempSect+1][0].y)<<endl;
                    if (abs(tempDoor[0].x-cornerStruct[tempSect+1][0].x)<FEATURE_DOOR_XY_THRESHOLD || abs(tempDoor[0].y-cornerStruct[tempSect+1][0].y)<FEATURE_DOOR_XY_THRESHOLD){
                        cout<<"Find a door defined by one flag corner. It's in "<<tempSect<<" and next section\n";
                        door.push_back(tempDoor[0]);
                        door.push_back(cornerStruct[tempSect+1][0]);
                        cornerStruct[tempSect+1][0].flag = 2; // reset flag
                    }
                }
            }
            else {
                cout<<"Not find satisfied door\n";
            }
            break;
        default:
            cout<<"Case 0\n";
            for (int i=0; i<sectNum-1; i++){
                double tempLastIndex = cornerStruct[i].size()-2;
                double tempdist = Feature::pointsDist(cornerStruct[i][tempLastIndex],cornerStruct[i+1][0]);
                cout<<"Gap distance is: "<<tempdist<<endl;
                if (tempdist>FEATURE_DOOR_MIN && tempdist<FEATURE_DOOR_MAX){
                    tempDoorCorner.flag = 2;
                    tempDoorCorner.x = cornerStruct[i].back().x; // last point of pre section
                    tempDoorCorner.y = cornerStruct[i].back().y;
                    door.push_back(tempDoorCorner);
                    tempDoorCorner.x = cornerStruct[i+1][0].x; // first point of current section
                    tempDoorCorner.y = cornerStruct[i+1][0].y;
                    door.push_back(tempDoorCorner);
                }
            }
            printf("Find %d door in corridor.\n",int(door.size()/2));
            break;
        }
    }
    else if (mode == "Room"){
        cout<<"Start find door inside room\n";
        for (int i=0;i<sectNum-1;i++){
            for (int j=0;j<cornerStruct[i].size();j++){
                if (cornerStruct[i][j].flag == 2){           // first check flag, if there is 2, better
                    cout<<"Find a exit corner\n";
                    tempDoorCorner.x = cornerStruct[i][j].x;
                    tempDoorCorner.y = cornerStruct[i][j].y;
                    tempDoor.push_back(tempDoorCorner);
                    tempSect = i;
                }
            }
        }
    }

    for (int i=0;i<door.size();i++){
        cout<<"The coordinates of door are: "<<door[i].x<<","<<door[i].y<<endl;
    }
    cout<<"findDoor function works fine!\n";
    return door;
}

// ****************************************** calculate 2 points distance ***********************************
double Feature::pointsDist(Feature::CornerInfo point1, Feature::CornerInfo point2){
    double dist;
    dist = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
    return dist;
}

double Feature::pointsDist(double x1, double y1, double x2, double y2){
    double dist;
    dist = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    return dist;
}

// *************************************** get line parameter a b c *****************************************
Feature::linePar Feature::getLinePar(double x1, double y1, double x2, double y2){
    Feature::linePar linePar;
    linePar.a = (y1-y2)/(x1-x2);
    linePar.b = -1;
    linePar.c = (x1*y2-x2*y1)/(x1-x2);
    linePar.aa = linePar.a*linePar.a;
    linePar.bb = linePar.b*linePar.b;
    return linePar;
}

// *********************************** point to line dist **************************************************
double Feature::pointLineDist(double x, double y, Feature::linePar linePar){
    double dis = abs(linePar.a*x + linePar.b*y + linePar.c)/sqrt(linePar.aa+linePar.bb);
    return dis;
}
