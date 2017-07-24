#include "PIDTunner.h"
#include "math.h"
#include <cmath>
#include <iostream>

using namespace std;
/*
* TODO: Complete the PID class.
*/

PIDTunner::PIDTunner() {}

PIDTunner::~PIDTunner() {}

void PIDTunner::Init(PID &pid, int error_index) {

    //I want to run 1 lap, around 5000m, and I choose distance because i want to minimize total cte for
    //steering controller and lap time for throttle controller
    tune_distance = 5000;

    //assume that data update comes 10 times per second;
    dt = 0.1;

    distance = 0;
    previous_cte = 0;
    previous_speed = 0;

//    cte_error = 0;
//    lap_time = 0;
//
//    min_cte_error = DBL_MAX;
//    min_lap_time = DBL_MAX;

    this->error_index = error_index;

    p = {pid.Kp, pid.Kd, pid.Ki};
    dp = {1, 1, 1};

    min_errors = {1000000000000, 1000000000000};
    total_errors = {0, 0};

    param_index = 0;  // was 2  this will wrao back to 0 after the first twiddle loop
    tried_adding = false;
    tried_subtracting = false;

}

void PIDTunner::UpdateError(PID &pid, double cte, double speed, uWS::WebSocket <uWS::SERVER> ws) {

    pid.UpdateError(cte);

    total_errors[0] += pow(cte, 2);
    total_errors[1] += dt;

    if(UpdateDistance(cte, speed) >= tune_distance || abs(cte) > 2.5){
        if(abs(cte) > 2.5){
            cout << "CTE TOO BIG" << endl;
            total_errors = {1000000000000 - distance, 1000000000000 - total_errors[1]};//std::numeric_limits<double>::max()

            cout << total_errors[error_index] << endl;
            cout << min_errors[error_index] << endl;

        }

        if (total_errors[error_index] < min_errors[error_index]) {
            cout << "Improvement!" << endl;
            cout << p[0] << " " << p[1] << " " << p[2] << endl;
            cout << dp[0] << " " << dp[1] << " " << dp[2] << endl;

            min_errors = total_errors;
            dp[param_index] *= 1.1;

            // next parameter
            param_index = (param_index + 1) % 3;
            tried_adding = tried_subtracting = false;
        }
        if (!tried_adding && !tried_subtracting) {
            // try adding dp[i] to params[i]
            p[param_index] += dp[param_index];
            tried_adding = true;
        }
        else if (tried_adding && !tried_subtracting) {
            // try subtracting dp[i] from params[i]
            p[param_index] += -2 * dp[param_index];
            tried_subtracting = true;
        }
        else {
            // set it back, reduce dp[i], move on to next parameter
            p[param_index] += dp[param_index];
            dp[param_index] *= 0.9;
            // next parameter
            param_index = (param_index + 1) % 3;
            tried_adding = tried_subtracting = false;
        }

        ResetPID(pid);
        RestartSimulator(ws);

    }
}

double PIDTunner::TotalError(PID &pid) {
    return pid.TotalError();
}

void PIDTunner::ResetPID(PID &pid) {
    pid.Init(p[0], p[2], p[1]);
    total_errors = {0, 0};
    distance = 0;
    previous_cte = 0;
    previous_speed = 0;

    cout << "CURRENT" << endl;
    cout << p[0] << " " << p[1] << " " << p[2] << endl;
    cout << dp[0] << " " << dp[1] << " " << dp[2] << endl;
    cout << "CURRENT CONTROLLER" << endl;
    cout << pid.Kp << " " << pid.Kd << " " << pid.Ki << endl;
}

void PIDTunner::RestartSimulator(uWS::WebSocket<uWS::SERVER> ws) {
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

double PIDTunner::UpdateDistance(double cte, double speed) {
    distance += sqrt(pow(cte - previous_cte, 2) + pow((previous_speed + speed) / 2 * 0.44704 * dt, 2));
    previous_cte = cte;
    previous_speed = speed;
    return distance;



    double min_speed;


}



