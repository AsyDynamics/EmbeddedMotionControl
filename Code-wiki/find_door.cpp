// **************************************** find door ******************************************************
bool Feature::findDoor(string mode, vector<vector<Feature::CornerInfo> > &cornerStruct){ // use mode, 2D vector of point struct as input
    vector<Feature::CornerInfo> tempDoor;
    vector<Feature::CornerInfo> door;
    int tempSect;
    int num2Corner = 0;
    Feature::CornerInfo tempDoorCorner;
    tempDoor.clear();
    int sectNum = cornerStruct.size();
    if (mode =="Corridor" || mode == "Room"){
        for (int i=0;i<sectNum;i++){
            for (int j=0;j<cornerStruct[i].size();j++){
                if (cornerStruct[i][j].flag == 2){    // first check flag, if there is 2, cheers. the exit point is ideal to define a door
                    num2Corner+=1;
                    tempDoorCorner.x = cornerStruct[i][j].x;
                    tempDoorCorner.y = cornerStruct[i][j].y;
                    tempDoor.push_back(tempDoorCorner);
                    tempSect = i;
                }
            }
        }
        switch(tempDoor.size()){ // check the number of exit point
        case 2:
            double doorWidth;
            doorWidth = pointsDist(tempDoor[0],tempDoor[1]); // calculate the distance between two exit point
            if (doorWidth>FEATURE_DOOR_MIN && doorWidth<FEATURE_DOOR_MAX){ // compare the distance with the real door dimension
                door.push_back(tempDoor[0]);
                door.push_back(tempDoor[1]);
            }
            break;
        case 1: // there is only one exit point found
            double doorWidth1;
            cout<<tempSect<<endl;
            doorWidth1 = pointsDist(tempDoor[0],cornerStruct[tempSect-1].back()); // previous segment's last point
            double doorWidth2;
            doorWidth2= pointsDist(tempDoor[0],cornerStruct[tempSect+1][0]);      // next segment's initial point
            if (doorWidth1>FEATURE_DOOR_MIN && doorWidth1<FEATURE_DOOR_MAX){
                if (tempSect>0){  // for safety
                    if (abs(tempDoor[0].x-cornerStruct[tempSect-1].back().x)<FEATURE_DOOR_XY_THRESHOLD || abs(tempDoor[0].y-cornerStruct[tempSect-1].back().y)<FEATURE_DOOR_XY_THRESHOLD){
                        door.push_back(tempDoor[0]);
                        door.push_back(cornerStruct[tempSect-1].back());
                        cornerStruct[tempSect-1].back().flag = 2; // reset flag
                    }
                }
            }
            if(doorWidth2>FEATURE_DOOR_MIN && doorWidth2<FEATURE_DOOR_MAX){ // cannot use else if, because sometimes these two satisfies at same time, may miss door
                if (tempSect+1<sectNum){ // for safety
                    cout<<abs(tempDoor[0].x-cornerStruct[tempSect+1][0].x)<<","<<abs(tempDoor[0].y-cornerStruct[tempSect+1][0].y)<<endl;
                    if (abs(tempDoor[0].x-cornerStruct[tempSect+1][0].x)<FEATURE_DOOR_XY_THRESHOLD || abs(tempDoor[0].y-cornerStruct[tempSect+1][0].y)<FEATURE_DOOR_XY_THRESHOLD){
                        door.push_back(tempDoor[0]);
                        door.push_back(cornerStruct[tempSect+1][0]);
                        cornerStruct[tempSect+1][0].flag = 2; // reset flag
                    }
                }
            }
            break;
        default:
            for (int i=0; i<sectNum-1; i++){
                double tempLastIndex = cornerStruct[i].size()-2;
                double tempdist = Feature::pointsDist(cornerStruct[i][tempLastIndex],cornerStruct[i+1][0]);
                if (tempdist>FEATURE_DOOR_MIN && tempdist<FEATURE_DOOR_MAX){
                    tempDoorCorner.flag = 2;
                    tempDoorCorner.x = cornerStruct[i].back().x; // last point of pre segment
                    tempDoorCorner.y = cornerStruct[i].back().y;
                    door.push_back(tempDoorCorner);
                    tempDoorCorner.x = cornerStruct[i+1][0].x;   // first point of current segment
                    tempDoorCorner.y = cornerStruct[i+1][0].y;
                    door.push_back(tempDoorCorner);
                }
            }
            break;
        }
    }
    return true;
}