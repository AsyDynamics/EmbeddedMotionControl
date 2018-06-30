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
    std::vector<std::vector<Feature::CornerInfo> > cornerStruct;
    std::vector<std::vector<Feature::CornerInfo> > Map;
    std::string filename = "NewMap";
    filename.append(".csv");
    Feature::bias bias; // bias correction from perception
    bias.x = 0;
    bias.y = 0;
    bias.theta = 0;

    cornermatrix.clear();
    Map.clear();
    std::string Corridor = "Corridor";
    std::string Room = "Room";
    double y1 = -0.4; // this should be boundary of corridor, first point's y of first scan
    double y2 = 0.4; // this should be boundary of corridor, last point's y of first scan


    // ************************************* teleop to scan ***********************************
    int control=1;

    while(control!=0){
        std::cout<<"Instruction: 100-initial scan, 1-scan, feature detection; 2-index2Struct, Corridor mode; 3-index2Struct, Room mode; 0-break, save map\n";
        std::cin>>control;
        if (control==100){
            if(aquireAveragedData(32, flaser, &io, &r)){
                std::cout<<"Initial scan\n";
            }
            Claser = feature.Preprocess(flaser,bias);
            y1 = Claser.vy[0];
            y2 = Claser.vy.back();
            cout<<"Boudary of corridor: "<<y1<<","<<y2<<endl;
        }

        if (control==1){
            if(aquireAveragedData(32, flaser, &io, &r)){
                std::cout<<"Acquared laser data!\n";
                std::cout<<"Acquired Odom position: "<<flaser.x<<","<<flaser.y<<","<<flaser.theta<<std::endl;
            }
            Claser = feature.Preprocess(flaser,bias);
            sect_info = feature.FindSection(Claser);
            cornermatrix = feature.FeatureDetection2(Claser,sect_info);
        }
        if (control==2){
            cornerStruct = feature.index2Struct(Corridor,cornermatrix,Claser,y1,y2);
            feature.findDoor(Corridor,cornerStruct);
            feature.MergeMap(cornerStruct,Map);
        }
        if (control==3){
            cornerStruct = feature.index2Struct(Room,cornermatrix,Claser,y1,y2);
            feature.findDoor(Corridor,cornerStruct);
            feature.MergeMap(cornerStruct,Map);
        }

        if (control==0){
            std::cout<<"Out of teleop! \nStarting save map data!\n";
            if(feature.saveMap(filename.c_str(), Map)){
                std::cout<<"Map data saved!\n";
            }
        }
    }

    return 0;
}


