#ifndef PID_ABSTRACTTUNNER_H
#define PID_ABSTRACTTUNNER_H

#include <vector>

#include "AbstractController.h"

class ParametersTunner {
public:

    AbstractController* controller;

    double tune_distance;
    double dt;
    double distance;
    double previous_speed;
    double previous_cte;

    std::vector<double> dp;
    std::vector<double> p;

    std::vector<double> best_p;


    double min_cost;
    double current_cost;

    int param_index, param_count;
    int step, steps;
    bool tried_adding, tried_subtracting, need_restart, success_lap, improved;

    int error_index;

    /*
    * Constructor
    */
    ParametersTunner();

    /*
    * Destructor.
    */
    virtual ~ParametersTunner();

    /*
    * Initialize Tunner.
    */
    virtual void Init(AbstractController* controller, std::vector<double> p, std::vector<double> dp);

    /*
    * Update the PID error variables given cross track error.
    */

    double ControlSignal(double cte, double speed, bool tune);

    void Reset();

    double UpdateDistance(double cte, double speed);

};

#endif /* PID_ABSTRACTTUNNER_H */
