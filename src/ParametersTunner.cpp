#include "ParametersTunner.h"
#include "math.h"
#include "cmath"
#include <iostream>

static const long long int MAX_COST = 1000000000000;
using namespace std;

ParametersTunner::ParametersTunner() {
    steps = 20;
}

ParametersTunner::~ParametersTunner() {}

void ParametersTunner::Init(AbstractController* controller, std::vector<double> p, std::vector<double> dp, bool tune) {

    this->controller = controller;

    this->controller->Init(p);

    this->tune = tune;

    //I want to run 1 lap, around 5000m, and I choose distance because i want to minimize total cte for
    //steering controller and lap time for throttle controller
    tune_distance = 5000;
    tune_time = 2000;

    //assume that data update comes 10 times per second;
    dt = 0.1;

    distance = 0;
    time = 0;

    previous_cte = 0;
    previous_speed = 0;

    this->p = p;
    this->dp = dp;
    this->best_p = p;

    this->error_index = error_index;

    min_cost = {MAX_COST, MAX_COST};
    current_cost = {0, 0};

    param_index = 0;
    param_count = this->p.size();
    tried_adding = false;
    tried_subtracting = false;

    step_without_improvement = 0;
}

double ParametersTunner::ControlSignal(double cte, double speed, double angle) {
    double control = controller->ControlSignal(cte, speed, angle);

    current_cost[0] += pow(cte, 2);
    current_cost[1] += abs(100 - speed);

    time++;

    if(tune && ( time >= tune_time || abs(cte) > 3.5 || speed < -0.1)){//UpdateDistance(cte, speed) >= tune_distance

        if(abs(cte) > 3.5 ){
            cout << "CTE TOO BIG" << endl;
            // When car can't finish lap it still have to be able to track progress
            current_cost = {MAX_COST - distance, MAX_COST - distance};
        }

        if( speed <  -0.1){
            cout << "GOING BACKWARD" << endl;
            current_cost = {MAX_COST - distance, MAX_COST - distance};
        }

        if (current_cost[controller->cost_index] < min_cost[controller->cost_index]) {
            cout << "Improvement!" << endl;

            step_without_improvement = 0;
            best_p = p;

            min_cost = current_cost;
            dp[param_index] *= 1.1;

            // next parameter
            param_index = (param_index + 1) % param_count;
            while(dp[param_index] <= 0.000001){
                param_index = (param_index + 1) % param_count;
            }

            tried_adding = tried_subtracting = false;
        } else {
            step_without_improvement++;
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

            // don't try to tune parameters with small dp
            while(dp[param_index] <= 0.000001){
                param_index = (param_index + 1) % param_count;
            }

            tried_adding = tried_subtracting = false;

            p[param_index] += dp[param_index];
            tried_adding = true;
        }

        need_restart = true;

    }
    return control;
}

void ParametersTunner::Reset() {
    controller->Init(p);
    current_cost = {0, 0};
    distance = 0;
    time = 0;
    previous_cte = 0;
    previous_speed = 0;

    need_restart = false;

    std::cout << "p" << endl;
    for (auto i: p)
        std::cout << i << ' ';
    std::cout << endl;

}

double ParametersTunner::UpdateDistance(double cte, double speed) {
    distance += sqrt(pow(cte - previous_cte, 2) + pow((previous_speed + speed) / 2 * 0.44704 * dt, 2));
    previous_cte = cte;
    previous_speed = speed;
    return distance;
}


void ParametersTunner::PrintParams() {
    std::cout << "p" << endl;
    for (auto i: best_p)
        std::cout << i << ' ';
    std::cout << endl;

    std::cout << "dp" << endl;
    for (auto i: dp)
        std::cout << i << ' ';
    std::cout << endl;
}
