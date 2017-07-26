#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;

    useNforI = false;
    windowSize = 10;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    if(useNforI){
        queue.push(cte);
        if(queue.size() > windowSize){
            queue.pop();
        }

        i_error = 0;
        std::queue<double> tmp_q = queue; //copy the original queue to the temporary queue

        while (!tmp_q.empty())
        {
            i_error += tmp_q.front();
            tmp_q.pop();
        }
    }

}

double PID::TotalError() {
    return - (Kp*p_error) - (Kd*d_error) - (Ki*i_error);
}