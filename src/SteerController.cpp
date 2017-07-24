#include <cmath>
#include "SteerController.h"

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