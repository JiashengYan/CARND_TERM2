#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

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
  }
  else if (b1 != string::npos && b2 != string::npos) {
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
  bool twiddle = false;
  bool init = true;
  bool direction_changed = false;
  double p[3] = {0.2, 0.0001, 3.8}; // initial pid parameter before twiddle
  double dp[3] = {0.02, 2.5e-7, .2};  // initial dp before twiddle
  int n = 0;
  int min_n = 100;
  int max_n = 6000;
  double total_cte = 0.0;
  double best_cte = 10000000.00;
  double tol = 0.001;
  int p_iter = 0;
  int total_iterator = 0;
  double best_p[3] = {p[0],p[1],p[2]};  //store best pid parameter
  if(twiddle == true) {
    pid.Init(best_p[0],best_p[1],best_p[2]);
  }else {
    pid.Init(0.229, 0.000100928, 3.6976);
  }

  h.onMessage([&pid, &twiddle, &init, &direction_changed, &p, &dp, &n, &max_n, &min_n, &total_cte, &best_cte, &tol, &p_iter, &total_iterator, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double steer_value;
          double throttle_value = 0.3;
          json msgJson;
          /**
           * TODO: Calculate steering value here, remember the steering value is [-1, 1].
           */
          n += 1;
          if(twiddle == true){
            if(n < min_n){//before drive into twiddle part of road
              //Steering value
              pid.UpdateError(cte);
              steer_value = pid.TotalError();
              if(n % 100 == 1){
                std::cout << "before twiddle" << std::endl;
              }
            }else if(n > max_n){//drive out of twiddle part of road, reset simulator
              n = 0;
              pid.Init(p[0],p[1],p[2]);
              std::cout << "reset" << " ";
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }else if(n == min_n){// twiddle parameter
              if(init == true){// first time run using dp
//                p[p_iter] += dp[p_iter];
                init = false;
                direction_changed = false;
                total_cte = 0;
                std::cout << "----- ----- ----- ----- INIT" << std::endl;
              }else{//  comparing previous cte with best result
                std::cout << "error: " << total_cte/(max_n-min_n) << std::endl;
                std::cout << "best_error: " << best_cte/(max_n-min_n) << std::endl;
                if(total_cte < best_cte){// get better result
                  std::cout << "----- ----- ----- ----- Better CTE" << std::endl;
                  std::cout << "----- ----- Best p[0] p[1] p[2]: " << p[0] << "---" << p[1] << "---" << p[2] << std::endl;
                  double sumdp = abs(dp[0]) + abs(dp[1]) + abs(dp[2]);
                  if(sumdp < tol) {
                    std::cout << "Best p[0] p[1] p[2]: " << p[0] << "---" << p[1] << "---" << p[2] << std::endl;
                    twiddle = false;
                    std::string reset_msg = "42[\"reset\",{}]";
                    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                  }
                  dp[p_iter] *= 1.1;
                  best_cte = total_cte;
                  best_p[0] = p[0];
                  best_p[1] = p[1];
                  best_p[2] = p[2];
                  p_iter = (p_iter > 1) ? 0 : p_iter+1; // move to next parameter
                  std::cout << "----- ----- change to parameter: " << p_iter << std::endl;
                  if(p_iter == 0){
                    total_iterator +=1;
                    std::cout << "iteration: " << total_iterator << std::endl;
                    std::cout << "error: " << total_cte/(max_n-min_n) << std::endl;
                    std::cout << "best_error: " << best_cte/(max_n-min_n) << std::endl;
                    std::cout << "Best p[0] p[1] p[2]: " << p[0] << "---" << p[1] << "---" << p[2] << std::endl;
                  }
                  p[p_iter] += dp[p_iter];
                  pid.UpdateParameter(p[0],p[1],p[2]);
//                  init = true;
                  direction_changed = false;
                  
                }else if(direction_changed == false){// worst result, need change direction
                  std::cout << "----- ----- ----- ----- change twiddle direction" << std::endl;
                  p[p_iter] -= dp[p_iter];
                  dp[p_iter] *= -1;
                  p[p_iter] += dp[p_iter];
                  pid.UpdateParameter(p[0],p[1],p[2]);
                  direction_changed = true;
                  std::cout << "Twiddle dp[0] dp[1] dp[2]: " << dp[0] << "---" << dp[1] << "---" << dp[2] << std::endl;
                  std::cout << "----- ----- Current p[0] p[1] p[2]: " << p[0] << "---" << p[1] << "---" << p[2] << std::endl;
                  
                }
                else{// worst result, need to reduce dp
                  std::cout << "----- ----- ----- ----- reduce twiddle" << std::endl;

                  p[p_iter] -= dp[p_iter];  // restore previous parameter
                  dp[p_iter] *= 0.8;
                  direction_changed = false;
                  
                  p_iter = (p_iter > 1) ? 0 : p_iter+1; // move to next parameter
                  std::cout << "----- ----- change to parameter: " << p_iter << std::endl;
                  if(p_iter == 0){
                    total_iterator +=1;
                    std::cout << "iteration: " << total_iterator << std::endl;
                  }
                  p[p_iter] += dp[p_iter];
                  pid.UpdateParameter(p[0],p[1],p[2]);
                  std::cout << "Twiddle dp[0] dp[1] dp[2]: " << dp[0] << "---" << dp[1] << "---" << dp[2] << std::endl;
                  std::cout << "----- ----- Current p[0] p[1] p[2]: " << p[0] << "---" << p[1] << "---" << p[2] << std::endl;
                }
                total_cte = 0;
              }
              pid.UpdateError(cte);
              steer_value = pid.TotalError();
            }else{// run twiddled parameter on the road from n_min to n_max
              if(n % 100 == 1){
                std::cout << "run twiddled pid" << std::endl;
              }
              pid.UpdateError(cte);
              total_cte = total_cte + pow(cte,2);
              steer_value = pid.TotalError();
            }
          }else{//no twiddle
            //Steering value
//            std::cout << "NO twiddle" << std::endl;
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
          }
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
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
