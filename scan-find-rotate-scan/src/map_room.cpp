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
#include "openspace.cpp"



const std::string currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

    return buf;
}


bool aquireAveragedData(int scans, Openspace::AveragedLaserData &data, emc::IO *io, emc::Rate *r) {
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

    std::cout << "Initialize controller\n";
    Controller controller(&io);
    Controller controller2(&io);
    std::cout << "Start gathering data front\n";

    //************************************* declare variable *****************************************
    Openspace openspace;
    Openspace::AveragedLaserData flaser;
    Openspace::AveragedLaserData blaser;
    Openspace::AveragedLaserData Claser;  // the processed data within one scan
    Openspace::AveragedLaserData CClaser; // the processed data with two scan
    std::vector <double> openindexf;
    std::vector <double> openindexfull;




    //************************************* front scan first *****************************************
    if(aquireAveragedData(32, flaser, &io, &r)){
        std::cout << "Aquired averaged data from front side.\n";
    }


    //************************************* process data *********************************************
    // use preprocess instead of MergeData, because Mergedata needs two input

    Claser = openspace.Preprocess(flaser);
    double RawNum = sizeof(flaser.range)/sizeof(double);
    std::cout<<"Raw data size is "<<RawNum<<std::endl;
    std::cout<<"Processed data size is "<<Claser.vrange.size()<<std::endl;
    openindexf = openspace.FindGap(Claser); //

//********************************************* First scan no door **************************************************
    //if (openindex.back()>Claser.vangle.size()-1){

    if (openindexf[0]==-1){
        //std::cout<<"Something Wrong! Openindex exceeds the data range!"<<std::endl;
        std::cout<<"Qualified gap not found! Need rotate and scan!"<<std::endl;
        // rotate, scan, data process
        std::cout<<"Not find the door at first attemp"<<std::endl;

        // rotate, scan
        std::cout << "Start rotating\n";
        controller.setRelativeSetpoint(0,0,PI);
        while(!controller.done){
            controller.control();
        }
        std::cout << "Stop rotating\n";

        // scan get back data
        if(aquireAveragedData(32, blaser, &io, &r)){
            std::cout << "Aquired averaged data from back side.\n";
        }

        // process data
        CClaser = openspace.MergeData(flaser,blaser);
        openindexfull = openspace.FindGap(CClaser);

        // check if the openindex exceed the point range

        if (openindexfull.back()>CClaser.vangle.size()-1){
            std::cout<<"Something Wrong! Really?"<<std::endl;
            std::cout<<"Openindexfull las value is: "<<openindexfull.back()<<std::endl;
            std::cout<<"Laser data range is: "<<CClaser.vangle.size()<<std::endl;
        }

        else{
            std::cout<<"The odom xy at first scan is: "<<flaser.x<<" , "<<flaser.y<<std::endl;
            std::cout<<"The odom xy at second scan is "<<blaser.x<<" , "<<blaser.y<<std::endl;
            double robotx = (flaser.x+blaser.x)/2;
            double roboty = (flaser.y+blaser.y)/2;

            std::cout<<"Start calculating openpoints' xy coordinates"<<std::endl;

            Openspace::OpenCoord openxy = openspace.OpenspaceCoord(openindexfull, CClaser, robotx, roboty);

            std::cout<<"The x coord of openspace is: "<<openxy.x[0]<<" and y is: "<<openxy.y[0]<<std::endl;
            std::cout<<"The x coord of openspace is: "<<openxy.x.back()<<" and y is: "<<openxy.y.back()<<std::endl;

            // drive to the point
//            if (openindexfull.size()>2){
//                double mx = (openxy.x[0]+openxy.x.back())/2;
//                double my = (openxy.y[0]+openxy.y.back())/2;
//            }
//            else{
//                double mx = (openxy.x[0]+openxy.x[1])/2;
//                double my = (openxy.y[0]+openxy.y[1])/2;
//            }
            double middlex = (openxy.x[0]+openxy.x.back())/2;
            double middley = (openxy.y[0]+openxy.y.back())/2;
            std::cout<<"The middle point coordinate is: "<<middlex<<" , "<<middley<<std::endl;


            controller2.setRelativeSetpoint(middlex, middley, 0);
            while(!controller2.done){
               controller2.control();
            }
        }

    }

//************************************** First scan find door, go there ******************************************
    else{
        // Find the door, calculate coordinate, go to the point
        std::cout<<"Find the door at first attemp and the openindex size is: "<<openindexf.size()<<std::endl;
        double robotx = flaser.x;
        double roboty = flaser.y;
        std::cout<<"The initial odom xy is: "<<robotx<<" , "<<roboty<<std::endl;
        Openspace::OpenCoord openxy = openspace.OpenspaceCoord(openindexf, Claser, robotx, roboty);
        std::cout<<"The x coord of openspace is: "<<openxy.x[0]<<" and y is: "<<openxy.y[0]<<std::endl;
        std::cout<<"The x coord of openspace is: "<<openxy.x.back()<<" and y is: "<<openxy.y.back()<<std::endl;

        // drive to the point
//        if (openindexf.size()>2){
//            double mx = (openxy.x[0]+openxy.x.back())/2;
//            double my = (openxy.y[0]+openxy.y.back())/2;
//        }
//        else{
//            double mx = (openxy.x[0]+openxy.x[1])/2;
//            double my = (openxy.y[0]+openxy.y[1])/2;
//        }
        double middlex = (openxy.x[0]+openxy.x.back())/2;
        double middley = (openxy.y[0]+openxy.y.back())/2;
        std::cout<<"The middle point coordinate is: "<<middlex<<" , "<<middley<<std::endl;
        controller.setRelativeSetpoint(middlex, middley, 0);
        while(!controller.done){
            controller.control();
        }
    }
    //std::cout<<"The last openindex-Claser.range is "<<openindex.back()-Claser.vangle.size()<<std::endl;



    return 0;
}

