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
#include "openspace.h"
//#include "openspace.cpp"


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
    std::vector<double> vrange;
    std::vector<double> vangle;
    double x;
    double y;
    double theta;
};



bool saveData(const char *file, const AveragedLaserData &data) {

    std::ofstream filestream(file);

    if (filestream.is_open()) {
        filestream << data.x << "," << data.y << "," << data.theta << "\n";
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

    //emc::LaserData raw_scan;
    AveragedLaserData filtered_scan;
    std::string filename;
    std::string dateTime=currentDateTime();

    std::cout << "Initialize controller\n";
    Controller controller(&io);
    std::cout << "Start gathering data front\n";

    if(aquireAveragedData(32, filtered_scan, &io, &r)){
        //filename = "data/front_data_";
        //filename.append(dateTime).append(".csv");
        //saveData(filename.c_str(), filtered_scan);
    } else {
        std::cout << "Could not aquire averaged data from front side.\n";
    }
    std::cout << "Stopped gathering data front\n";

    std::cout << "Start rotating\n";
    controller.setRelativeSetpoint(0,0,PI);
    while(!controller.done){
        controller.control();
    }
    std::cout << "Stop rotating\n";

    std::cout << "Start gathering back front\n";
    if(aquireAveragedData(32, filtered_scan, &io, &r)){
        //filename = "data/back_data_";
        //filename.append(dateTime).append(".csv");
        //saveData(filename.c_str(), filtered_scan);
    } else {
        std::cout << "Could not aquire averaged data from back side.\n";
    }
    std::cout << "Stopped gathering data back\n";

    return 0;
}
