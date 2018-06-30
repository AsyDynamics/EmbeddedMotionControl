#ifndef FEATURE
#define FEATURE
#include "config.h"
class Feature{
private:

public:
    emc::IO *io;
    emc::OdometryData odom;


    struct linePar{
        double a;
        double b;
        double c;
        double aa;
        double bb;
    };

    struct section_index{
        std::vector<double> init; // index of initial point in section
        std::vector<double> end;
        std::vector<double> temp; // the updated initial point, used in FeatureDetection function
        double num; // number of secitons in total
    };

    struct CornerInfo{
        double index;
        double x;
        double y;
        double flag; // 0-temp corner, 1-room(inner) corner, 2-door(outer) corner, -1 unknown
    };

    struct AveragedLaserData {
        int succesfulScans[SCAN_POINTS];
        double range[SCAN_POINTS];
        double angle[SCAN_POINTS];
        std::vector<double> vrange;
        std::vector<double> vangle;
        std::vector<double> vx;
        std::vector<double> vy;
        double x;
        double y;
        double theta;
    };

    bool aquireAveragedData(int scans, AveragedLaserData &data, emc::IO *io, emc::Rate *r);
    AveragedLaserData Preprocess(AveragedLaserData &laser);
    AveragedLaserData MergeScan(AveragedLaserData &flaser, AveragedLaserData &blaser);
    section_index FindSection(AveragedLaserData &laser);
    //CornerInfo FeatureDetection(AveragedLaserData &laser, section_index &section_info);
    std::vector<std::vector<double> > FeatureDetection(AveragedLaserData &laser, section_index &section_info);
    bool saveMap(const char *file, const std::vector<std::vector<double> > &Mapxy);
    bool saveMap(const char *file, const std::vector<std::vector<CornerInfo> > cornerStruct);
    bool MergeMap(std::vector<std::vector<double> > &Mapxy, std::vector<std::vector<double> > &cornermatrix, AveragedLaserData &laser);
    bool findDoor(std::string mode, std::vector<std::vector<CornerInfo> > cornerStruct);
    double pointsDist(CornerInfo point1, CornerInfo point2);
    double pointsDist(double x1, double y1, double x2, double y2);
    linePar getLinePar(double x1, double y1, double x2, double y2);
    double pointLineDist(double x, double y, linePar linePar);
    std::vector<std::vector<CornerInfo> > index2Struct(std::vector<std::vector<double> > cornermatrix, AveragedLaserData &laser);
};

#endif // FEATURE

