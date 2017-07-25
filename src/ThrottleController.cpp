#include "ThrottleController.h"

#include <math.h>
#include <cmath>

using namespace std;

double ThrottleController::ControlSignal(double cte, double speed){
    pid.UpdateError(cte);

    double min_speed = k[3];
    double min_throttle = k[4];
    double throttle_shift = k[5];

    double throttle = abs(throttle_shift) -  abs(pid.TotalError());//min(1,);

    if(throttle < -1)
        throttle = -1;

    if(throttle > 1)
        throttle = 1;

    if(speed < abs(min_speed))
        throttle =  abs(min_throttle);

    return throttle;
}

double ThrottleController::Cost(double cte, double speed){
    return 0.1;
}