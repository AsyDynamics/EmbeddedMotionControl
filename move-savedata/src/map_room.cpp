#include <emc/rate.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cstring>
#include <time.h>
#include <unistd.h>
#include "controller.h"
#include "controller.cpp"

const std::string currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

    return buf;
}

struct AveragedLaserData {
    int succesfulScans[SCAN_POINTS];
    double range[SCAN_POINTS];
    double angle[SCAN_POINTS];
    // odom data
    double x;
    double y;
    double theta;
    // control effort data
    double cex;
    double cey;
    double cetheta;
};

bool saveData(const char *file, const AveragedLaserData &data) {

    std::ofstream filestream(file);

    if (filestream.is_open()) {
        filestream << data.x << "," << data.y << "," << data.theta << "\n";
        filestream << data.cex << "," << data.cey << "," << data.cetheta << "\n";
        for (int i = 0; i < SCAN_POINTS; i++) {
            filestream << data.succesfulScans[i] << "," << data.angle[i] << "," << double(data.range[i]) << "\n";
        }
        filestream.close();
        return true;
    } else {
        std::cout << "Could not write to file!\n";
        return false;
    }
}

bool aquireAveragedData(int scans, AveragedLaserData &data, emc::IO *io, emc::Rate *r) {
    if (scans <= 0)
        return false;

    emc::LaserData rawScan;
    emc::OdometryData odom;
    emc::ControlEffort effort;

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

    while(io->ok()){
    std::cout << "testing ce" << std::endl;
        if(io->readControlEffort(effort)){
            std::cout << "read ce!" << std::endl;
            data.cex = effort.x;
            data.cey = effort.y;
            data.cetheta = effort.th;
            break;
        }
        r->sleep();
    }

    return true;
}


int main() {
    emc::IO io;

    emc::Rate r(20);
    emc::ControlEffort effort;
    emc::OdometryData odom;
    Controller controller(&io);

    //emc::LaserData raw_scan;
    AveragedLaserData filtered_scan;
    std::string filename;
    std::string dateTime=currentDateTime();

    // control effort
    while(io.ok()){
        if(io.readControlEffort(effort)){
            std::cout << "read control effort: " << effort.x<<" , "<<effort.y<<" , "<<effort.th<<std::endl;
//            break;
        }
        else{
//            std::cout<<"Read effort fail.\n";
        }
        r.sleep();
    }

    //******************************************* initial scan, no move ********************************************************
    std::cout<<"Start gathering data step1.\n";
    if(aquireAveragedData(32, filtered_scan, &io, &r)){
        filename = "data/step1_data_";
        filename.append(dateTime).append(".csv");
        saveData(filename.c_str(), filtered_scan);
        std::cout<<"Odom coord: "<<filtered_scan.x<<" , "<<filtered_scan.y<<" , "<<filtered_scan.theta<<std::endl;
        std::cout<<"ControEfford : "<<filtered_scan.cex<<" , "<<filtered_scan.cey<<" , "<<filtered_scan.cetheta<<std::endl;
        std::cout<<"Accquared.\n";
    } else {
        std::cout << "Could not aquire averaged data.\n";
    }
    std::cout << "Stopped gathering data\n"<<"Start move.\n";


    // ***************************************** move, x+0.5, scan *************************************************************
    std::cout<<"Start to move step 2.\n";
    controller.setRelativeSetpoint(1, 1, 0);
    while(!controller.done){
        controller.control();
    }

    std::cout<<"Start gathering data.\n";
    if(aquireAveragedData(32, filtered_scan, &io, &r)){
        filename = "data/step2_data_";
        filename.append(dateTime).append(".csv");
        saveData(filename.c_str(), filtered_scan);
        std::cout<<"Odom coord: "<<filtered_scan.x<<" , "<<filtered_scan.y<<" , "<<filtered_scan.theta<<std::endl;
        std::cout<<"ControEfford : "<<filtered_scan.cex<<" , "<<filtered_scan.cey<<" , "<<filtered_scan.cetheta<<std::endl;
        std::cout<<"Accquared.\n";
    } else {
        std::cout << "Could not aquire averaged data.\n";
    }
    std::cout << "Stopped gathering data\n";

    // ***************************** move, rotate 180 deg, counterclockwise, scan **********************************

    std::cout<<"Start to move step 3.\n";
    controller.setRelativeSetpoint(0, 0, PI);
    while(!controller.done){
        controller.control();
    }

    std::cout<<"Start gathering data.\n";
    if(aquireAveragedData(32, filtered_scan, &io, &r)){
        filename = "data/step3_data_";
        filename.append(dateTime).append(".csv");
        saveData(filename.c_str(), filtered_scan);
        std::cout<<"Odom coord: "<<filtered_scan.x<<" , "<<filtered_scan.y<<" , "<<filtered_scan.theta<<std::endl;
        std::cout<<"ControEfford : "<<filtered_scan.cex<<" , "<<filtered_scan.cey<<" , "<<filtered_scan.cetheta<<std::endl;
    } else {
        std::cout << "Could not aquire averaged data.\n";
    }
    std::cout << "Stopped gathering data\n"<<"Finish.\n";

    return 0;
}
