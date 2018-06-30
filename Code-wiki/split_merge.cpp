// ******************************************** Feature Detection, overcome missing corner **********************************
vector<vector<double> > Feature::FeatureDetection2(Feature::AveragedLaserData &laser, Feature::section_index &section_info){ // processed laser & odom data and segment info as input
    vector<double> cornerindex;
    vector<vector<double> > cornermatrix;
    Feature::linePar linePar1, linePar2, linePar3; // assume in worst case we could find less than three feature point within one segment ()
    int tempIndex1, tempIndex2, tempIndex3;        // temporarily save the point with max distance, could be corner or just point in a wall
    for (int i = 0; i<section_info.num;i++){       // loop among segments
        cornerindex.push_back(section_info.init[i]);
        int init_index = section_info.init[i];
        int end_index  = section_info.end[i];
        linePar1 = getLinePar(laser.vx[init_index],laser.vy[init_index],laser.vx[end_index],laser.vy[end_index]); // get the parameters a, b, c of line
        double maxdis = 0;
        for (int k1 = section_info.init[i]+1; k1<section_info.end[i]-1;k1++){  // loop of points in each segment
            double dis1 = pointLineDist(laser.vx[k1],laser.vy[k1],linePar1);   // calculate the distance of each point to the line
            if (dis1>maxdis){ // find the max distance, save the distance and its corresponding point index temporarily
                maxdis = dis1;
                tempIndex1 = k1;
            }
        }
        if (maxdis>FEATURE_WALL){ // compare the found max distance with threshold of wall
            cornerindex.push_back(tempIndex1);
            linePar2 = getLinePar(laser.vx[init_index],laser.vy[init_index],laser.vx[tempIndex1],laser.vy[tempIndex1]); // to avoid missing possible feature point, fit two lines
            linePar3 = getLinePar(laser.vx[end_index],laser.vy[end_index],laser.vx[tempIndex1],laser.vy[tempIndex1]);
            maxdis = 0; 
            for (int k2 = section_info.init[i]+1; k2<tempIndex1-1; k2++){
                double dis2 = pointLineDist(laser.vx[k2], laser.vy[k2],linePar2);
                if(dis2>maxdis){
                    maxdis = dis2;
                    tempIndex2 = k2;
                }
            }
            if(maxdis>FEATURE_WALL){
                cornerindex.push_back(tempIndex2);
            }
            maxdis = 0; 
            for (int k3 = tempIndex1+1; k3<section_info.end[i]-1; k3++){
                double dis3 = pointLineDist(laser.vx[k3], laser.vy[k3], linePar3);
                if(dis3>maxdis){
                    maxdis = dis3;
                    tempIndex3 = k3;
                }
            }
            if(maxdis>FEATURE_WALL){
                cornerindex.push_back(tempIndex3);
            }
        }
        cornerindex.push_back(section_info.end[i]);
        sort(cornerindex.begin(),cornerindex.end()); // the vector should contain all the feature point with monatic increasing index
        cornermatrix.push_back(cornerindex);         // push every row to the 2D vector
        cornerindex.clear();
    }
    return cornermatrix;
}