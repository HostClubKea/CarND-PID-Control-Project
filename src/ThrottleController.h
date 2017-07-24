#ifndef PID_THROTTLECONTROLLER_H
#define PID_THROTTLECONTROLLER_H


#include "AbstractController.h"

class ThrottleController : public AbstractController {

    double ControlSignal(double cte, double speed) override;
};


#endif //PID_THROTTLECONTROLLER_H
