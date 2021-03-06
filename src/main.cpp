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

  PID steer_pid;
  PID throttle_pid;
  /**
   * TODO: Initialize the pid variable.
   */
  unsigned int twiddle_update_counter = 0;
  unsigned int update_params_idx = 0;
  unsigned int update_params_step = 0;
  bool twiddle_update = false;
  std::vector<double> params = {0.171, 0.000015, 1.0305};
  std::vector<double> d_params = {0.01, 0.001, 0.5};
  // std::vector<double> params = {0.160192, 0.005482, 1.692654};
  // std::vector<double> d_params = {0.003, 0.000045, 0.024901};
  std::vector<double> best_params = params;
  std::vector<double> best_d_params = d_params;
  steer_pid.Init(params[0], params[1], params[2]);
  double min_score = std::numeric_limits<double>::infinity();
  const double min_tolerance = 0.01;
  const double bad_cte = 4.4;
  const int circle_step = 1000;

  const double target_speed = 38.5;
  std::vector<double> params_2 = {20.0, 0.00005, 8.0};
  throttle_pid.Init(params_2[0], params_2[1], params_2[2]);

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
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          double throttle = 0.3;
          double steer_value;

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // Compute throttle
          double speed_cte = speed - target_speed;
          throttle_pid.UpdateError(speed_cte);
          throttle_pid.control_value(throttle);

          if (twiddle_update) {
            // Tune PID hyperparameters by twiddle method
            double early_stop = (fabs(cte) > bad_cte) ? true : false;
            if (early_stop || twiddle_update_counter == circle_step) {
              if (early_stop) {
                printf("Too bad, reset: %f\n", cte);
              }
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            } else {
              steer_pid.UpdateError(cte);
              steer_pid.control_value(steer_value);
              twiddle_update_counter++;
              /* printf("counter: %4d, cte: %10.5f, error: %12.5f, min_score: "
                     "%12.5f\n",
                     twiddle_update_counter, cte, pid.TotalError(), min_score);
               */

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              // std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            // Normal flow
            steer_pid.UpdateError(cte);
            steer_pid.control_value(steer_value);

            // DEBUG
            // printf("steer: %6.3f, speed: %4.2f, throttle: %4.1f\n",
            // steer_value, speed, throttle);
            /* std::cout << "CTE: " << cte << " Steering Value: " <<
               steer_value << std::endl; */
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        } // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    if (twiddle_update && (update_params_idx % params.size() == 0)) {
      double tolerance = d_params[0] + d_params[1] + d_params[2];
      printf("tolerance: %f\n", tolerance);
      if (tolerance <= min_tolerance) {
        // Stop twiddle
        twiddle_update = false;
        printf("Stop twiddle\n");
        printf("Final params: %f, %f, %f\n", best_params[0], best_params[1],
               best_params[2]);
      }
    }

    if (twiddle_update) {
      printf("\nupdate_params_idx: %d\n", update_params_idx);
      printf("update_params_step: %d\n", update_params_step);
      printf("d_params: %f, %f, %f\n", d_params[0], d_params[1], d_params[2]);
      printf("Current min_score: %f\n", min_score);
      printf("Current best params: %f, %f, %f\n", best_params[0],
             best_params[1], best_params[2]);

      switch (update_params_step) {
      case 0: {
        auto temp = params;
        params[update_params_idx] += d_params[update_params_idx];
        printf("0 params: %f, %f, %f -> %f, %f, %f\n", temp[0], temp[1],
               temp[2], params[0], params[1], params[2]);
        update_params_step++;
        break;
      }
      case 1: {
        double error = fabs(steer_pid.TotalError());
        double score = error / (twiddle_update_counter);
        printf("error: %f, score: %f\n", error, score);
        if (score < min_score) {
          printf("best error: %f -> %f\n", min_score, score);
          min_score = score;
          best_params = params;
          best_d_params = d_params;
          d_params[update_params_idx] *= 1.1;

          // For next parameter index, step 0
          update_params_idx = (update_params_idx + 1) % params.size();
          auto temp = params;
          params[update_params_idx] += d_params[update_params_idx];
          printf("1 params: %f, %f, %f -> %f, %f, %f\n", temp[0], temp[1],
                 temp[2], params[0], params[1], params[2]);
          update_params_step = 1;
        } else {
          auto temp = params;
          params[update_params_idx] -= 2 * d_params[update_params_idx];
          printf("2 params: %f, %f, %f -> %f, %f, %f\n", temp[0], temp[1],
                 temp[2], params[0], params[1], params[2]);
          update_params_step++;
        }
        break;
      }
      case 2: {
        double error = fabs(steer_pid.TotalError());
        double score = error / (twiddle_update_counter);
        printf("error: %f, score: %f\n", error, score);
        if (score < min_score) {
          printf("best error: %f -> %f\n", min_score, score);
          min_score = score;
          best_params = params;
          best_d_params = d_params;
          d_params[update_params_idx] *= 1.1;
        } else {
          auto temp = params;
          params[update_params_idx] += d_params[update_params_idx];
          printf("3 params: %f, %f, %f -> %f, %f, %f\n", temp[0], temp[1],
                 temp[2], params[0], params[1], params[2]);
          d_params[update_params_idx] *= 0.9;
        }
        update_params_idx = (update_params_idx + 1) % params.size();
        auto temp = params;
        params[update_params_idx] += d_params[update_params_idx];
        printf("4 params: %f, %f, %f -> %f, %f, %f\n", temp[0], temp[1],
               temp[2], params[0], params[1], params[2]);
        update_params_step = 1;
        break;
      }
      default:
        printf("Twiddle update error!\n");
        break;
      }

      steer_pid.Init(params[0], params[1], params[2]);
      twiddle_update_counter = 0;
    }
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