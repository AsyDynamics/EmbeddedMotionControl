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
    std::string filename = "MapPoint";
    filename.append(".csv");

    Controller control(&io);

    control.setRelativeSetpoint(-2,0,PI/6);
    while(!control.done){
        control.control();
    }

    //************************************* Split and Merge **************************************
    if(aquireAveragedData(32, flaser, &io, &r)){
        //std::cout << "Aquired averaged data from front side.\n";
        //std::cout<<"Aquired laser size is: "<<flaser.vrange.size()<<std::endl;
    }

    Claser = feature.Preprocess(flaser);
    std::cout<<"Processed Laser size: "<<Claser.vangle.size()<<std::endl;
    //std::cout<<"The first vangle after Preprocess is: "<<Claser.vangle[0]<<std::endl;
    //std::cout<<"The vx is: "<<Claser.vx[0]<<std::endl;
    sect_info = feature.FindSection(Claser);
    cornermatrix = feature.FeatureDetection(Claser,sect_info);

    std::cout<<"Go back to main function!\n";
    double temp1=cornermatrix.size();
    std::cout<<"The section number in total is: "<<temp1<<std::endl;
    for (double k=0;k<temp1;k++){
        std::cout<<"\nThis is "<<k<<" section. The corner points index:\n";
        double temp2 = cornermatrix[k].size();
        for (double j=0;j<temp2;j++){
            std::cout<<cornermatrix[k][j]<<",";
        }
    }
    std::cout<<"\nMain function finish.\n";
    bool fileflag = saveMap(filename.c_str(),cornermatrix,Claser);
    return 0;
}

