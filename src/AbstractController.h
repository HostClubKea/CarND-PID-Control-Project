#ifndef ABSTRACT_CONTROLLER_H
#define ABSTRACT_CONTROLLER_H

#include "PID.h"
#include <vector>

class AbstractController {
public:

    PID pid;

    std::vector<double> k;

    int cost_index;
    /*
    * Constructor
    */
    AbstractController();

    /*
    * Destructor.
    */
    virtual ~AbstractController();

    /*
    * Initialize Controller.
    */
    void Init(std::vector<double> k);

    /*
   * Get Control signal from controller.
   */
    virtual double ControlSignal(double cte, double speed, double angle) = 0;
};

#endif /* ABSTRACT_CONTROLLER_H */
