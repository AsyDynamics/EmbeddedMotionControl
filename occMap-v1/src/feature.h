#ifndef FEATURE
#define FEATURE
class Feature{
private:

public:
    emc::IO *io;
    emc::OdometryData odom;

    struct section_index{
        std::vector<double> init; // index of initial point in section
        std::vector<double> end;
        std::vector<double> temp; // the updated initial point, used in FeatureDetection function
        double num;               // number of secitons in total
    };

    struct CornerInfo{
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> index;
        std::vector<double> flag;
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


    AveragedLaserData Preprocess(AveragedLaserData &laser);
    AveragedLaserData MergeScan(AveragedLaserData &flaser, AveragedLaserData &blaser);
    section_index FindSection(AveragedLaserData &laser);
    std::vector<std::vector<double> > FeatureDetection(AveragedLaserData &laser, section_index &section_info);
    bool saveMap(const char *file, const std::vector<std::vector<double> > &Mapxy);
    bool MergeMap(std::vector<std::vector<double> > &Mapxy, std::vector<std::vector<double> > &cornermatrix, AveragedLaserData &laser);

    bool updateOccupancyMap(double OccMap[FEATURE_MAP_X/FEATURE_MAP_R][FEATURE_MAP_Y/FEATURE_MAP_R], section_index &sect_info, AveragedLaserData &Claser);
    bool saveOccupancyMap(const char *file, const double OccMap[FEATURE_MAP_X/FEATURE_MAP_R][FEATURE_MAP_Y/FEATURE_MAP_R]);
};

#endif // FEATURE

