#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

 // Tuning settings
 // How far in front of you a car needs to be to be considered too close.
 const uint too_close_distance = 30;
 // How far in front of you a car needs to be to make a lane change beind it.
 // This should be greater than the too_close distance to minimize continuous
 // lane changes.
 const uint lane_front_space = 50;
 // How far behind you a car needs to be for you to make a lane change in 
 // front of it. This prevents cutting a car off.
 const uint lane_behind_space = 5; 
 // Number of waypoints to generate in the spline. A lower value is more 
 // responsive but more stable.
 const uint spline_points = 25;



  // Have a reference velocity to target
  static double ref_vel = 0.0; // mph
  // Start in lane 1; lane 0 is far left, lane 2 is far right
  static uint lane = 1; 
  static uint counter = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          uint number_of_lanes = 2; // number of lanes - 1

          // Counter to see if there are any vehicles blocking either of the 
          // adjacent lanes.
          uint left_lane_blocked = 0; 
          uint right_lane_blocked = 0;

 
          // Find ref_v to use
          for (uint i = 0; i < sensor_fusion.size(); i++) {
            // Car is in my lane
            float d = sensor_fusion[i][6]; // list of lanes of all cars
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            
            // if using previous points, can project s value out
            check_car_s += ((double) prev_size * 0.02 * check_speed); 
 
            if ((d < (2 + 4 * lane + 2)) && 
                (d > (2 + 4 * lane - 2)) &&
                (check_car_s > car_s) && 
                ((check_car_s - car_s) < too_close_distance)) {
              // For all cars in my lane
              // Check s values greater than mine and s gap
              too_close = true;
            }
            uint right_of_right_lane = (2 + 4 * (lane + 1) + 2); 
            uint left_of_right_lane = (2 + 4 * (lane + 1) - 2); 
            uint right_of_left_lane = (2 + 4 * (lane - 1) + 2); 
            uint left_of_left_lane = (2 + 4 * (lane - 1) - 2);

            // Check lane change possibilities
            if (((check_car_s - car_s) < lane_front_space) &&
                (check_car_s > (car_s - lane_behind_space)) && 
                (d < right_of_left_lane) &&                                         
                (d > left_of_left_lane)) {
              left_lane_blocked++; 
            }
            if (((check_car_s - car_s) < lane_front_space) &&
                (check_car_s > (car_s - lane_behind_space)) && 
                (d < right_of_right_lane) &&
                (d > left_of_right_lane)) {
              right_lane_blocked++; 
            } 
          }

          if (too_close) {
            // the total acceleration should not exceed 10 m/s^2
            ref_vel -= 0.224; // 0.224
              if (counter >= 60) { // counter prevents rapid lane changes
              if (lane > 0 && lane < number_of_lanes) {
                if (left_lane_blocked == 0) {                  
                  lane -= 1;
                  counter = 0;
                }
                else if (right_lane_blocked == 0) { 
                  lane += 1; 
                  counter = 0;
                }
              } else if (lane == 0) {
                if (right_lane_blocked == 0) { 
                  lane += 1; 
                  counter = 0;
                }
              } else if (lane == number_of_lanes) {
                if (left_lane_blocked == 0) { 
                  lane -= 1; 
                  counter = 0;
                }
              }
            }
          } else if (ref_vel < 48.5) {
            ref_vel += (0.224);
          }

          counter++;
          
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, and yaw states
          // either reference the starting poing as where the car is 
          // or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if(prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else { // Use the previous path's end point as starting reference
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Use two points that make the path tangent to the previous 
            // path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In frenet coordinates, add evenly 30m spaced points ahead of 
          // the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, 
                                          (2 + 4 * lane), 
                                          map_waypoints_s, 
                                          map_waypoints_x, 
                                          map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 
                                          (2 + 4 * lane), 
                                          map_waypoints_s, 
                                          map_waypoints_x, 
                                          map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 
                                          (2 + 4 * lane), 
                                          map_waypoints_s, 
                                          map_waypoints_x, 
                                          map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (uint i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // Create a spline
          tk::spline s;

          // set (x, y) points to the spline
          s.set_points(ptsx, ptsy);

          // Define the actual (x, y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start the new path points with the previous path points
          for (uint i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our 
          // desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with 
          // previous points, here we will always output 50 points
          
          for (uint i = 1; i <= spline_points - previous_path_x.size(); i++) { // 50
            // Calculate the points along the spline that correlate to 
            // approximately 50 MPH. The 2.24 is due to this being in m/s not MPH.
            double N = (target_dist / (.02 * ref_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier (shift and rotation)
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          /*  Using information from the previous path ensures that there is a 
           *  smooth transition from cycle to cycle. But the more waypoints we 
           *  use from the previous path, the less the new path will reflect 
           *  dynamic changes in the environment.
           *
           *  Ideally, we might only use a few waypoints from the previous path 
           *  and then generate the rest of the new path based on new data from 
           *  the car's sensor fusion information.
           *  */

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
