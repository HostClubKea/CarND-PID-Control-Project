#ifndef PID_STEERCONTROLLER_H
#define PID_STEERCONTROLLER_H


#include "AbstractController.h"

class SteerController : public AbstractController {

    double ControlSignal(double cte, double speed) override;
};


#endif //PID_STEERCONTROLLER_H
