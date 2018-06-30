#include "controller.h"
#include "config.h"
#include <cmath>
#include <emc/io.h>
#include <emc/rate.h>
#include <unistd.h>

//Controller::Controller(*io) {
//}

void Controller::checkAngle() {
    while (std::fabs(target_a) > PI) {
        if (std::signbit(target_a))
            target_a += 2*PI;
        else
            target_a -= 2*PI;
    }
    this ->control();
}

void Controller::control() {
    // Check if setpoint has been achieved, otherwise adjust control output
    if(done){
        std::cout << "Made it to setpoint, exiting controll loop\n";
        return;
    }
    while(io->ok()){
        if(io->readOdometryData(odom)){
            // Continue checking here.
            vel_x = (target_x-odom.x);
            vel_y = (target_y-odom.y);
            vel_a = (target_a-odom.a);
            if(std::fabs(vel_a) > PI)
                vel_a = -vel_a;

            vel_x = (std::fabs(vel_x) < CONTROLLER_ACCURACY) ? 0 : vel_x*CONTROLLER_P_TRANS;
            vel_y = (std::fabs(vel_y) < CONTROLLER_ACCURACY) ? 0 : vel_y*CONTROLLER_P_TRANS;
            vel_a = (std::fabs(vel_a) < CONTROLLER_ACCURACY) ? 0 : vel_a*CONTROLLER_P_ROT;

            this->saturate();

            io->sendBaseReference(vel_x,vel_y,vel_a);
            if(std::fabs(vel_x) < 1e-9 && std::fabs(vel_y) < 1e-9 && std::fabs(vel_a) < 1e-9)
                done = true;
            break;
        }
    }
}

void Controller::setSetpoint(const double &x, const double &y, const double &a) {
    done = false;
    target_x = x;
    target_y = y;
    target_a = a;
    this->checkAngle();
    //this->control();
}

void Controller::setRelativeSetpoint(const double &x, const double &y, const double &a) {
    done = false;
    target_x += x;
    target_y += y;
    target_a += a;
    this->checkAngle();
    //this->control();
}

void Controller::getSetpoint(int &x, int &y, int &a) {
    x = target_x;
    y = target_y;
    a = target_a;
}

void Controller::saturate() {
    vel_a = std::min(std::max(vel_a, -SAT_LIM_ROT), SAT_LIM_ROT);
    double scaling = std::pow(vel_x,2) + std::pow(vel_y,2);
    if (scaling > SAT_LIM_TRANS2){
        scaling = std::sqrt(SAT_LIM_TRANS/(std::pow(vel_x,2)+std::pow(vel_y,2)));
        vel_x = scaling * vel_x;
        vel_y = scaling * vel_y;
    }
}
