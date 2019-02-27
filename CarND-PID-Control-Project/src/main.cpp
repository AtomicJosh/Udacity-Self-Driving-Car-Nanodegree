#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;

  /**
   * Initialize PID and twiddle variables
   */

  bool twiddle_on = false;
  double p[3] = {0.05, 0.0001, 1.5};
  double dp[3] = {.01, .0001, .1};
  int n = 0;
  int max_n = 600;
  double cte_sum = 0.0;
  double error = 0.0;
  double best_error = 100.0;
  double tolerance = 0.001;
  int p_i = 0;
  int total_i = 0;
  int sub_move = 0;
  double best_p[3] = {p[0],p[1],p[2]};
  bool first_pass = true;
  bool second_pass = true;
  if(twiddle_on == true) { pid.Init(p[0],p[1],p[2]); }
  else { pid.Init(0.111051, 0.0005641, 1.7979); }
  h.onMessage([&pid, &p, &dp, &n, &max_n, &tolerance, &error, &best_error, &p_i, &total_i, &cte_sum, &first_pass, &sub_move, &second_pass, &twiddle_on, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double throttle_value = 0.3;
          double steer_value;
          json msgJson;
          
          if (twiddle_on == true){
            cte_sum += pow(cte,2);
            if (n==0) { pid.Init(p[0],p[1],p[2]); }
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            
            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
            n = n+1;
            if (n > max_n) { 
              if(first_pass == true) {
                p[p_i] += dp[p_i];
                first_pass = false;
              } else {
                  error = cte_sum/max_n;
                
                  if(error < best_error && second_pass == true) {
                    best_error = error;
                    best_p[0] = p[0];
                    best_p[1] = p[1];
                    best_p[2] = p[2];
                    dp[p_i] *= 1.1;
                    sub_move += 1;
                    std::cout << "Iteration: " << total_i << "\n";
                    std::cout << "Error: " << error << "\n";
                    std::cout << "Best Error: " << best_error << "\n";
                    std::cout << "Best p: " << best_p[0] << " " 
                                            << best_p[1] << " " 
                                            << best_p[2] << "\n";
                } else {
                    if(second_pass == true) {
                      p[p_i] -= 2 * dp[p_i];
                      second_pass = false;
                    } else {
                      if(error < best_error) {
                        best_error = error;
                        best_p[0] = p[0];
                        best_p[1] = p[1];
                        best_p[2] = p[2];
                        dp[p_i] *= 1.1;
                        sub_move += 1;
                      } else {
                        p[p_i] += dp[p_i];
                        dp[p_i] *= 0.9;
                        sub_move += 1;
                      }
                    std::cout << "Iteration: " << total_i << "\n";
                    std::cout << "Error: " << error << "\n";
                    std::cout << "Best Error: " << best_error << "\n";
                    std::cout << "Best p: " << best_p[0] << " " 
                                            << best_p[1] << " " 
                                            << best_p[2] << "\n";
                  }
                }
              }
              
              if(sub_move > 0) {
                p_i += 1;
                first_pass = true;
                second_pass = true;
                sub_move = 0;
              }
              if(p_i == 3) { p_i = 0; }
              cte_sum = 0.0;
              n = 0;
              total_i += 1;

              if((dp[0] + dp[1] + dp[2]) < tolerance) {
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
                //ws.close();
              } else {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
              
            } else {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
        
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
