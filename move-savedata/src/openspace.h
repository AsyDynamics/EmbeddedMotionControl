#ifndef OPENSPACE
#define OPENSPACE
class Openspace {

private :
    //emc::IO *io;
    //emc::OdometryData odom;

public:
    /*Openspace(emc::IO *io) {
        inOut = io;
        emc::LaserData laser = emc::LaserData();        
	return;
    }
    */
    emc::IO *io;
    emc::OdometryData odom;

    struct OpenCoord{
        std::vector<double> x;
        std::vector<double> y;
    };

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

    //struct AveragedLaserData {
    //    int succesfulScans[SCAN_POINTS];
    //    double range[SCAN_POINTS];
    //    double angle[SCAN_POINTS];
        //std::vector<double> range;
        //std::vector<double> angle;

    //    double x;
    //    double y;
    //    double theta;
    //};


    // variables used
    // emc-defined type declare
    //AveragedLaserData flaser;           // front scan laser data
    //AveragedLaserData blaser;           // back scan laser data
    //std::vector<emc::OdometryData> odom;          // use its theta

    // self-defined type declare
    //AveragedLaserData ProcessedData; // check highlight
    //AveragedLaserData Claser;        //after merge
    
    // output coord of openspace in vector

    //std::vector<double> openx;
    //std::vector<double> openy;

    AveragedLaserData MergeData(AveragedLaserData &flaser, AveragedLaserData &blaser);
    AveragedLaserData Preprocess(AveragedLaserData &laser);
    std::vector<double> FindGap(AveragedLaserData &Claser);
    OpenCoord OpenspaceCoord(std::vector<double> &openindex, AveragedLaserData &Claser, double &robotx, double &roboty);
};

#endif // OPENSPACE

