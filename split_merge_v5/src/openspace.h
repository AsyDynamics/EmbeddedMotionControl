#ifndef OPENSPACE
#define OPENSPACE
class Openspace {

private :


public:

    emc::IO io;
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



    AveragedLaserData MergeData(AveragedLaserData &flaser, AveragedLaserData &blaser);
    AveragedLaserData Preprocess(AveragedLaserData &laser);
    std::vector<double> FindGap(AveragedLaserData &Claser);
    OpenCoord OpenspaceCoord(std::vector<double> &openindex, AveragedLaserData &Claser, double &robotx, double &roboty);
};

#endif // OPENSPACE

