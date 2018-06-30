#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "config.h"
#include <emc/io.h>
#include <emc/rate.h>

//TODO: make singleton of this class, such that only one controller can be active on the system.
class Controller {
public:
    Controller(emc::IO *inOut){
        io=inOut;
        vel_x = 0;
        vel_y = 0;
        vel_a = 0;
        while(io->ok()){
            if(io->readOdometryData(odom)){
                target_x = odom.x;
                target_y = odom.y;
                target_a = odom.a;
                done = true;
                break;
            } else {
                std::cout << "IO not initialized for initial position gathering.\n";
            }
            usleep(1000000);
        }
    }
    void getSetpoint(int &x, int &y, int &a);
    void setSetpoint(const double &x,const double &y,const double &a);
    void setRelativeSetpoint(const double &x,const double &y,const double &a);
    void control();
    bool done;

private:
    double vel_x;
    double vel_y;
    double vel_a;

    double target_x;
    double target_y;
    double target_a;

    void saturate();
    void checkAngle();

    emc::IO *io;
    emc::OdometryData odom;
    emc::LaserData scan;
    emc::ControlEffort effort;

};

#endif // CONTROLLER_H
