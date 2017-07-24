#ifndef PID_TUNNER_H
#define PID_TUNNER_H

#include "PID.h"
#include <vector>
#include <uWS/uWS.h>

class PIDTunner {
public:

    double tune_distance;
    double dt;
    double distance;
    double previous_speed;
    double previous_cte;

//    double cte_error;
//    double lap_time;
//    double min_cte_error;
//    double min_lap_time;

    std::vector<double> dp;
    std::vector<double> p;

    std::vector<double> min_errors;
    std::vector<double> total_errors;

    int param_index;
    bool tried_adding, tried_subtracting;

    int error_index;

    /*
    * Constructor
    */
    PIDTunner();

    /*
    * Destructor.
    */
    virtual ~PIDTunner();

    /*
    * Initialize PID.
    */
    void Init(PID &pid, int error_index);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(PID &pid, double cte, double speed, uWS::WebSocket <uWS::SERVER> ws);

    /*
    * Calculate the total PID error.
    */
    double TotalError(PID &pid);


    void ResetPID(PID &pid);

    void RestartSimulator(uWS::WebSocket <uWS::SERVER> ws);

    double UpdateDistance(double cte, double speed);
};

#endif /* PID_H */
