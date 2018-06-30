// ******************************************** build corner struct from corner index matrix ***************************************************
vector<vector<Feature::CornerInfo> > Feature::index2Struct(std::string mode, vector<vector<double> > cornermatrix, Feature::AveragedLaserData &laser, double y1, double y2){ // use mode, feature point info matrix, processed sensor data, boundary of corridor as input
    vector<vector<Feature::CornerInfo> > cornerStruct;
    Feature::CornerInfo tempCorner;
    vector<Feature::CornerInfo> row;
    for(int i=0;i<cornermatrix.size();i++){
        for(int j=0;j<cornermatrix[i].size();j++){
            if(j==0 || j==cornermatrix[i].size()-1){ // the flag of initial and end point is different from others
                tempCorner.flag = -1;
            }
            else {
                double corIndex  = cornermatrix[i][j];
                double preIndex  = corIndex - FEATURE_COMPARE;
                double postIndex = corIndex + FEATURE_COMPARE;
                double preDist   = Feature::pointsDist(laser.vx[preIndex],laser.vy[preIndex],laser.x,laser.y);   // distance of pico to previous n point
                double postDist  = Feature::pointsDist(laser.vx[postIndex],laser.vy[postIndex],laser.x,laser.y); // distance of pico to post n point
                double corDist   = Feature::pointsDist(laser.vx[corIndex],laser.vy[corIndex],laser.x,laser.y);   // distance of pico to the feature point
                if (preDist<corDist && postDist<corDist){       // point type: corner of room
                    tempCorner.flag = 1;
                }else if (preDist>corDist && postDist>corDist){ // point type: exit point
                    tempCorner.flag = 2;
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
    switch (flag){
    case 1:      // mode corridor
        for (int i=0;i<cornerStruct.size();i++){
            if (cornerStruct[i][0].y>0 && cornerStruct[i].back().y>0){      // both y coord of end point of segment is y>0 (left hand side)
                if (cornerStruct[i][0].y-y2>FEATURE_BOUND_THRESHOLD && cornerStruct[i].back().y-y2>FEATURE_BOUND_THRESHOLD){   // compare with boundary of cooridor
                    cornerStruct.erase(cornerStruct.begin()+i); // remove
                    i--;
                }
            }else if(cornerStruct[i][0].y<0 && cornerStruct[i].back().y<0){ // both y corrd end of segment is y<0 (right hand side)
                if (-cornerStruct[i][0].y+y1>FEATURE_BOUND_THRESHOLD && -cornerStruct[i].back().y+y1>FEATURE_BOUND_THRESHOLD){ // compare with boundary of cooridor
                    cornerStruct.erase(cornerStruct.begin()+i);
                    i--;
                }
            }
        }
        break;
    case 2: // mode room
        for (int i=0;i<cornerStruct.size();i++){
            for(int j=0;j<cornerStruct[i].size();j++){
                if (cornerStruct[i][j].flag==2){  // flag 2 means the exit point
                    tempCorner.x = cornerStruct[i][j].x;
                    tempCorner.y = cornerStruct[i][j].y;
                    tempCorner.flag = i;          // this flag is not the same as before, just to store section number used later
                    row.push_back(tempCorner);
                }
            }
        }
        switch(row.size()){ // the number of exit point
        case 2:
            if (row[1].flag-row[0].flag>1){ // if there is one or more segment between these two exit point
                int eraseMarker = row[0].flag+1;
                for (int i=row[0].flag;i<row[1].flag-1;i++){
                    cornerStruct.erase(cornerStruct.begin()+eraseMarker); // remove the middle segments
                }
            }
            break;
        case 3:
            int eraseMarker = row[0].flag+1;
            for (int i =row[0].flag;i<row[2].flag-1;i++){
                cornerStruct.erase(cornerStruct.begin()+eraseMarker);
            }
            break;
        }
        break;
    }
    return cornerStruct;
}