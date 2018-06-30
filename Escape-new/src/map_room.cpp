#include <emc/rate.h>
#include <emc/io.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cstring>
#include <time.h>
#include <unistd.h>
#include "monitoring.h"
#include "monitoring.cpp"


int main() {
    std::cout << "Start main!\n";

    //************************************* declare variable *****************************************
    emc::IO io;
    emc::Rate r(20);
    Feature feature;
    Feature::AveragedLaserData flaser;
    Feature::AveragedLaserData Claser;
    Feature::section_index sect_info;
    std::vector<std::vector<double> > cornermatrix;

    Feature::bias bias; // bias correction from perception
    bias.x = 0;
    bias.y = 0;
    bias.theta = 0;



    // ********************************* Scan and move ********************************************
    aquireAveragedData(32, flaser, &io, &r);
    Claser = feature.Preprocess(flaser,bias);
    sect_info = feature.FindSection(Claser);
    cornermatrix = feature.FeatureDetection2(Claser,sect_info);

    if (cornermatrix.size()>1){
        double index1 = cornermatrix[0].back();
        double index2 = cornermatrix.back()[0];
        double x1 = Claser.vx[index1];
        double x2 = Claser.vx[index2];
        double y1 = Claser.vy[index1];
        double y2 = Claser.vy[index2];
        double dist = feature.pointsDist(x1,y1,x2,y2);
        std::cout<<"Gap distance = "<<dist<<endl;
        if (dist>0.5){
            printf("Find door! Set point: %f ,%f\n",(x1+x2)/2, (y1+y2)/2);
// ***************************************** controller go here ******************************************************
            //control.run();
        }
    } else if(cornermatrix.size()==1){
        std::cout<<"Turn back, enter 1 and scan. The cin is just for teleop to turn back, should be removed by using controller\n";
        int Turn;
        std::cin>>Turn;
        if (Turn==1){
            aquireAveragedData(32, flaser, &io, &r);
            Claser = feature.Preprocess(flaser,bias);
            sect_info = feature.FindSection(Claser);
            cornermatrix = feature.FeatureDetection2(Claser,sect_info);
            if (cornermatrix.size()>1){
                double index1 = cornermatrix[0].back();
                double index2 = cornermatrix.back()[0];
                double x1 = Claser.vx[index1];
                double x2 = Claser.vx[index2];
                double y1 = Claser.vy[index1];
                double y2 = Claser.vy[index2];
                double dist = feature.pointsDist(x1,y1,x2,y2);
                std::cout<<"Gap distance = "<<dist<<endl;
                if (dist>0.5){
                    printf("Find door! Set point: %f ,%f\n",(x1+x2)/2, (y1+y2)/2);
// ***************************************** controller go here ******************************************************
                    //            control.run();
                }
            }
        }
    }
    else std::cout<<"Error"<<endl;

    // After go to the setpoint, still need to go further in order to exit the corridor completely.
    // Could set a far point based on initial coordinates and the previous setpoint\n";
    // ***************************************** controller go here ******************************************************
    //control.run();

    return 0;
}


