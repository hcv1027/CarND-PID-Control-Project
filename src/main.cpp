#include "PID.h"
#include "json.hpp"
#include <iostream>
#include <limits>
#include <math.h>
#include <string>
#include <uWS/uWS.h>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  double Kp = 0.2;
  double Ki = 0.004;
  double Kd = 3.0;
  // pid.Init(Kp, Ki, Kd);

  unsigned int step = 0;
  unsigned int update_params_idx = 0;
  unsigned int update_params_step = 0;
  bool twiddle_update = true;
  bool update_params = true;
  std::vector<double> params = {0.0, 0.0, 0.0};
  std::vector<double> d_params = {1.0, 1.0, 1.0};
  double best_error = std::numeric_limits<double>::infinity();
  auto twiddle = [&]() {};

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double throttle = 0.3;
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          if (twiddle_update) {
            if (step % 20 == 0) {
              switch (update_params_step) {
              case 0: {
                params[update_params_idx] += d_params[update_params_idx];
                pid.Init(params[0], params[1], params[2]);
                update_params_step = (update_params_step + 1) % 3;
                break;
              }
              case 1: {
                params[update_params_idx] += 2 * d_params[update_params_idx];
                pid.Init(params[0], params[1], params[2]);
                update_params_step = (update_params_step + 1) % 3;
                break;
              }
              case 2: {
                params[update_params_idx] += d_params[update_params_idx];
                d_params[update_params_idx] *= 0.9;
                pid.Init(params[0], params[1], params[2]);
                update_params_step = (update_params_step + 1) % 3;
                break;
              }

              default:
                break;
              }
            }
          }
          pid.control_value(throttle, steer_value);

          bool reset = false;
          step++;

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    printf("steer,cte,diff_cte,total_cte,p,d,i\n");
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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