#include "SteerController.h"
#include <cmath>
#include <iostream>


double SteerController::ControlSignal(double cte, double speed){
    pid.UpdateError(cte);
    double steer_value = pid.TotalError();

    if (steer_value > 1) {
        steer_value = 1;
    } else if (steer_value < -1) {
        steer_value = -1;
    }

    return steer_value;
}

double SteerController::Cost(double cte, double speed){
    return pow(cte, 2);
}

