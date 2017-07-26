#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
//#include "PID.h"
//#include "PIDTunner.h"
#include "SteerController.h"
#include "ParametersTunner.h"
#include "ThrottleController.h"

#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }


//I would have 2 PID controllers,
//1 for steering
//2 throttle
//I would use automatic parameters tuning
// For steering it would be minimizing total cte
// For throttle minimazing lap time
// lap is around 5 km


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;

    SteerController *steerController = new SteerController();
    steerController->cost_index = 0;
    ParametersTunner steerTunner;
    steerTunner.Init(steerController, {0.2012, 3.95312, 0.0001}, {0.02, 0.02, 0.0004}, false);

    ThrottleController *throttleController = new ThrottleController();
    throttleController->cost_index = 1;
    ParametersTunner throttleTunner;
    throttleTunner.Init(throttleController, {0.01, 0.76016, 0, 38, 1, 0.7055}, {0.01, 0.05, 0, 2, 0, 0.1}, false); //0.01 0.7


    h.onMessage([&steerTunner, &throttleTunner](uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length,
                                                uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event

        if (length && length > 2 && data[0] == '4' && data[1] == '2') {


            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());

                    double steer_value = steerTunner.ControlSignal(cte, speed, angle);
                    double throttle = throttleTunner.ControlSignal(cte, speed, angle);

                    if (throttleTunner.need_restart || steerTunner.need_restart) {
                        if (steerTunner.tune && steerTunner.step_without_improvement > steerTunner.steps) {
                            steerTunner.tune = false;
                            throttleTunner.tune = true;
                            steerTunner.p = steerTunner.best_p;
                            throttleTunner.step_without_improvement = 0;
                            throttleTunner.min_cost = steerTunner.min_cost;
                            steerTunner.steps += 10;

                            std::cout << "TUNE THROTTLE" << std::endl;

                            steerTunner.PrintParams();
                            throttleTunner.PrintParams();
                            throttleTunner.tried_adding = false;
                            throttleTunner.tried_subtracting = false;

                        }

                        if (throttleTunner.tune && throttleTunner.step_without_improvement > throttleTunner.steps) {
                            steerTunner.tune = true;
                            throttleTunner.tune = false;
                            throttleTunner.p = throttleTunner.best_p;
                            steerTunner.step_without_improvement = 0;
                            steerTunner.min_cost = throttleTunner.min_cost;
                            throttleTunner.steps += 10;

                            std::cout << "TUNE STEER" << std::endl;

                            steerTunner.PrintParams();
                            throttleTunner.PrintParams();

                            steerTunner.tried_adding = false;
                            steerTunner.tried_subtracting = false;
                        }

                        throttleTunner.Reset();
                        steerTunner.Reset();

                        std::string reset_msg = "42[\"reset\",{}]";
                        ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                    }


                    // DEBUG
                    //  std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
