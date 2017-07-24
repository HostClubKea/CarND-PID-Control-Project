#include "AbstractController.h"

AbstractController::AbstractController() {}

AbstractController::~AbstractController() {}

void AbstractController::Init(std::vector<double> k) {
    this->k = k;

    pid.Init(k[0], k[2], k[1]);
}

