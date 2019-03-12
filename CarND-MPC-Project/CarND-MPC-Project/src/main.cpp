#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

static int LATENCY = 100/1000; // latency in millisecond

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          double steer_value = j[1]["steering_angle"];
          double throttle_value= j[1]["throttle"];
          
          // convert to vehicle space
          Eigen::VectorXd ptsx_v = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsy_v = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
          for (int i = 0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx_v[i] = x * cos(psi) + y * sin(psi);
            ptsy_v[i] = - x * sin(psi) + y * cos(psi);
          }
          
          auto coeffs = polyfit(ptsx_v, ptsy_v, 3);
          double cte = - polyeval(coeffs, 0);
          double epsi = - atan(coeffs[1]);
          std::cout << "cte " << cte <<" epsi " << epsi << std::endl;
          
          Eigen::VectorXd state(6);
          
          const double Lf = 2.67;
          
          const double x0 = 0;
          const double y0 = 0;
          const double psi0 = 0;
          const double cte0 = coeffs[0];
          const double epsi0 = -atan(coeffs[1]);
          
          // State after delay.
          double x_delay = x0 + ( v * cos(psi0) * LATENCY );
          double y_delay = y0 + ( v * sin(psi0) * LATENCY );
          double psi_delay = psi0 - ( v * steer_value * deg2rad(25) * LATENCY / Lf );
          double v_delay = v + throttle_value * LATENCY;
          double cte_delay = cte0 + ( v * sin(epsi0) * LATENCY );
          double epsi_delay = epsi0 - ( v * steer_value * deg2rad(25) * LATENCY / Lf );
          
          state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;
          auto vars = mpc.Solve(state, coeffs);
          
          steer_value = - vars[0]/ deg2rad(25) ; // normalise - simulator has steering input [-1,1]
          if(steer_value > 1.0) steer_value = 1.0;
          if(steer_value < -1.0) steer_value = -1.0;
          throttle_value = vars[1];
          double N = vars[2];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
          for (int i = 0; i < N-1; i++) {
            mpc_x_vals.push_back(vars[3 + i]);
            mpc_y_vals.push_back(vars[3 + i + N]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */

          next_x_vals.resize(ptsx_v.size());
          next_y_vals.resize(ptsy_v.size());
          
          for (int i = 0; i < ptsx_v.size(); i++) {
            next_x_vals[i] = ptsx_v[i];
            next_y_vals[i] = ptsy_v[i];
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(LATENCY));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
