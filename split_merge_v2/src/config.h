#ifndef CONFIG
#define CONFIG

#define SCAN_POINTS 200
#define SCAN_STEPSIZE 5
#define SCAN_OFFSET 8
#define PI 3.141592654

#define SAT_LIM_TRANS 0.5 // [m/s]
#define SAT_LIM_TRANS2 0.25 // Squared saturation limit, saves square root computation time
#define SAT_LIM_ROT 1.2 // [rad/s]

#define CONTROLLER_P_TRANS 2 // Controller proportional gain
#define CONTROLLER_P_ROT 10 // Controller proportional gain
#define CONTROLLER_ACCURACY 0.01 // Controller accuracy (aka when is good enough)

//#define FEATURE_FAR 3.5 // Param in FindGap function, remove the far points, should larger than 3
#define FEATURE_GAP 0.5 // Param in FindGap funciton, find the gap larger than such distance.
#define FEATURE_OUTLIER 5 // if section has less than * points, these are outliers
#define FEATURE_WALL 0.1

//#define TRICK 2 // the distance from trick point to middle point

#endif // CONFIG

