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
    double time;
    double tune_time;

    std::vector<double> dp;
    std::vector<double> p;

    std::vector<double> best_p;

    std::vector<double> min_cost;
    std::vector<double> current_cost;

    int param_index, param_count;
    int step_without_improvement, steps;
    bool tried_adding, tried_subtracting, need_restart, tune; //success_lap, improved,

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
    virtual void Init(AbstractController* controller, std::vector<double> p, std::vector<double> dp, bool tune);

    /*
    * Update the PID error variables given cross track error.
    */

    double ControlSignal(double cte, double speed, double angle);

    void Reset();

    double UpdateDistance(double cte, double speed);

    void PrintParams();

};

#endif /* PID_ABSTRACTTUNNER_H */
