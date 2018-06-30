#include <emc/rate.h>
#include <emc/io.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cstring>
#include <time.h>
#include <unistd.h>
#include "controller.h"
#include "controller.cpp"
#include "feature.h"
#include "feature.cpp"



const std::string currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

    return buf;
}


bool aquireAveragedData(int scans, Feature::AveragedLaserData &data, emc::IO *io, emc::Rate *r) {
    if (scans <= 0)
        return false;

    emc::LaserData rawScan;
    emc::OdometryData odom;

    // Empty arrays such that we can fill them again
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
            std::cout<<"Odom xy coord is: "<<data.x<<", "<<data.y<<std::endl;
        }
        r->sleep();
    }
    return true;
}

int main() {
    emc::IO io;
    emc::Rate r(20);

    std::cout << "Start main!\n";

    //************************************* declare variable *****************************************
    Feature feature;
    Feature::AveragedLaserData flaser;
    Feature::AveragedLaserData Claser;
    Feature::section_index sect_info;
    std::vector<std::vector<double> > cornermatrix;
    std::vector<std::vector<double> > Mapxy;
    std::string filename = "OccMap-";
    filename.append(".csv");

    cornermatrix.clear();
    Mapxy.clear();

    double OccMap[FEATURE_MAP_X/FEATURE_MAP_R][FEATURE_MAP_Y/FEATURE_MAP_R]={};
    OccMap[0][0] = -1;
    OccMap[0][FEATURE_MAP_Y/FEATURE_MAP_R-1]  = -1;
    OccMap[FEATURE_MAP_X/FEATURE_MAP_R-1][0]  = -1;
    OccMap[FEATURE_MAP_X/FEATURE_MAP_R-1][FEATURE_MAP_Y/FEATURE_MAP_R-1] = -1;

    // ************************************* teleop to scan ***********************************
    int control=1;

    while(control!=0){
        std::cout<<"Enter any number to start, enter 0 to save and exit\n";
        std::cin>>control;

        if (control!=0){
            if(aquireAveragedData(32, flaser, &io, &r)){
                std::cout<<"Acquared laser data!\n";
                std::cout<<"Acquired Odom position: "<<flaser.x<<","<<flaser.y<<","<<flaser.theta<<std::endl;
            }
            Claser = feature.Preprocess(flaser);
            sect_info = feature.FindSection(Claser);
            feature.updateOccupancyMap(OccMap, sect_info, Claser);
        }
        else{
            std::cout<<"Out of teleop! \nStarting save map data!\n";
            feature.saveOccupancyMap(filename.c_str(),OccMap);
        }
    }
    return 0;
}

