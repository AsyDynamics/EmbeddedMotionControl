#include <emc/rate.h>
#include <emc/io.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cstring>
#include <time.h>
#include <unistd.h>
//#include "controller.h"
//#include "controller.cpp"
#include "feature.h"
#include "feature.cpp"


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
    std::vector<std::vector<double> > Mapxy;
    std::string filename = "NewMap";
    filename.append(".csv");

    cornermatrix.clear();
    Mapxy.clear();
    Map.clear();
    std::string mode = "Corridor";


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
            cornermatrix = feature.FeatureDetection(Claser,sect_info);
            cornerStruct = feature.index2Struct(cornermatrix,Claser);
            feature.findDoor(mode,cornerStruct);
            feature.MergeMap(cornerStruct,Map);
        }
        else{
            std::cout<<"Out of teleop! \nStarting save map data!\n";
            if(feature.saveMap(filename.c_str(), Map)){
                std::cout<<"Map data saved!\n";
            }
        }
    }

    return 0;
}


