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
        double num; // number of secitons in total
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
    AveragedLaserData Merge(AveragedLaserData &flaser, AveragedLaserData &blaser);
    section_index FindSection(AveragedLaserData &laser);
    //CornerInfo FeatureDetection(AveragedLaserData &laser, section_index &section_info);
    std::vector<std::vector<double> > FeatureDetection(AveragedLaserData &laser, section_index &section_info);
};

#endif // FEATURE

