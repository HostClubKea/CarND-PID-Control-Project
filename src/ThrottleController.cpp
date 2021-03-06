#include "ThrottleController.h"

#include <math.h>
#include <cmath>

using namespace std;

double ThrottleController::ControlSignal(double cte, double speed, double angle){
    pid.UpdateError(cte*speed/90);

    double min_speed = k[3];
    double min_throttle = k[4];
    double throttle_shift = k[5];

    double throttle = throttle_shift -  abs(pid.TotalError());//min(1,);

    if(throttle < -1)
        throttle = -1;

    if(throttle > 1)
        throttle = 1;

    if(speed < min_speed && throttle < min_throttle)
        throttle =  min_throttle;

    return throttle;
}
