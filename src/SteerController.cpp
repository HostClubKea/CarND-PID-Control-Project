#include "SteerController.h"
#include <cmath>
#include <iostream>


double SteerController::ControlSignal(double cte, double speed, double angle){
    pid.UpdateError(cte*speed/90);
    double steer_value = pid.TotalError();

    if (steer_value > 1) {
        steer_value = 1;
    } else if (steer_value < -1) {
        steer_value = -1;
    }

    return steer_value;
}

