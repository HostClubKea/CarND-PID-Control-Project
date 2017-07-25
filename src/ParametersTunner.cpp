#include "ParametersTunner.h"
#include "math.h"
#include "cmath"
#include <iostream>

using namespace std;

ParametersTunner::ParametersTunner() {}

ParametersTunner::~ParametersTunner() {}

void ParametersTunner::Init(AbstractController* controller, std::vector<double> p, std::vector<double> dp) {

    this->controller = controller;

    this->controller->Init(p);

    //I want to run 1 lap, around 5000m, and I choose distance because i want to minimize total cte for
    //steering controller and lap time for throttle controller
    tune_distance = 5000;

    //assume that data update comes 10 times per second;
    dt = 0.1;

    distance = 0;
    previous_cte = 0;
    previous_speed = 0;

    this->p = p;
    this->dp = dp;

    this->error_index = error_index;

    min_cost = 1000000000000;
    current_cost = 0;

    param_index = 0;  // was 2  this will wrao back to 0 after the first twiddle loop
    param_count = this->p.size();
    tried_adding = false;
    tried_subtracting = false;

}

double ParametersTunner::ControlSignal(double cte, double speed, bool tune) {
    double control = controller->ControlSignal(cte, speed);

    current_cost += controller->Cost(cte, speed);

    if(tune && (UpdateDistance(cte, speed) >= tune_distance || abs(cte) > 2.7)){
        if(abs(cte) > 2.7){
            cout << "CTE TOO BIG" << endl;

            current_cost = 1000000000000 - distance;
        }

        if (current_cost < min_cost) {
            cout << "Improvement!" << endl;

            for (auto i: p)
                std::cout << i << ' ';
            std::cout << endl;

            for (auto i: dp)
                std::cout << i << ' ';
            std::cout << endl;
//            cout << p[0] << " " << p[1] << " " << p[2] << endl;
//            cout << dp[0] << " " << dp[1] << " " << dp[2] << endl;

            min_cost = current_cost;
            dp[param_index] *= 1.1;

            // next parameter
            param_index = (param_index + 1) % param_count;
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
            param_index = (param_index + 1) % param_count;
            tried_adding = tried_subtracting = false;
        }

        need_restart = true;

    }
    return control;
}

void ParametersTunner::Reset() {
    controller->Init(p);
    current_cost = 0;
    distance = 0;
    previous_cte = 0;
    previous_speed = 0;

    need_restart = false;

    cout << "CURRENT" << endl;
    for (auto i: p)
        std::cout << i << ' ';
    std::cout << endl;

    for (auto i: dp)
        std::cout << i << ' ';
    std::cout << endl;
}

double ParametersTunner::UpdateDistance(double cte, double speed) {
    distance += sqrt(pow(cte - previous_cte, 2) + pow((previous_speed + speed) / 2 * 0.44704 * dt, 2));
    previous_cte = cte;
    previous_speed = speed;
    return distance;
}



