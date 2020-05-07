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

/// CONTROLLLER: Develop a basic controller that makes the velocity almost constant at 30 mph
double control_velocity(double speed,double angle){
  double throttle;
  if(speed >= 0 && speed <= 30){
    throttle = 0.3;
    // When curves are too closed then reduce the speed of the car
    if(abs(angle) >= 8){
      throttle = -0.015;
    }
  }
  // Stop pressing the throttle 
  else {
    throttle = 0.0;
  }
  return throttle;
}

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

// Variables for the twiddle
bool twiddle = false;
bool case_two = false;
int iterations =0;
int index_pos = 0;
int counter = 0;
double cummulative_error = 0.0;
double best_error = 9999;
double present_error = 0.0;
std::vector<double> best_score(3);
std::vector<double> p {0.105,0.0005,1.6};
std::vector<double> dp {0.01,0.00001, 0.2};

int main() {
  uWS::Hub h;

  PID pid;
  
  /**
   * TODO: Initialize the pid variable.
   */

  // Set the coefficients of the PID controller 
  double kp = 0.1381;   // Best values registered 0.04     0.117156     0.1381
  double ki = 0.000521; // Best values registered 0.0005   0.0075079    0.000521
  double kd = 0.856;    // Best values registered  1        1.4          1.18
  pid.Init(kp,ki,kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event.
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
          double steer_value; //Stores the steering value
          double throttle; //Stores the throttle value

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          // Tune the coefficients using the (twiddle = true)   
          if(twiddle){
            // If the number of iterations was satisfied or the error is too big at a given instant
            if(iterations >= 1000 || (present_error >= 3 && iterations >= 5)){
              // Calculate the cummulative (error plus a punish) 
              if (iterations < 1000){
                cummulative_error = cummulative_error / iterations + 1/iterations;
              }
              // The car completed the full iteration (pure error)
              else{
              cummulative_error = cummulative_error / iterations;
              }
              
              // Variable that stores the case depending the current error 
              int case_num;
              std::cout << " Total current error : " << cummulative_error << std::endl;
              std::cout << " Best_error : " << best_error << std::endl;
              
              // Case 1: current_error < best_error (nested if from case 4)
              if(cummulative_error < best_error && case_two == true){
                best_error = cummulative_error;
                dp[index_pos] *= 1.1;
                best_score[0] = p[0];
                best_score[1] = p[1];
                best_score[2] = p[2];
                case_num = 1;
              }
              // Case 2: current_error > best_error (nested else if from case 4)
              else if(cummulative_error > best_error && case_two == true){
                p[index_pos] += dp[index_pos];
                dp[index_pos]*= 0.9;
                case_num = 2;
              }
              // Case 3: current_error < best_error (first if)
              if(cummulative_error < best_error && case_two == false){
                best_error = cummulative_error;
                dp[index_pos] *= 1.1;
                best_score[0] = p[0];
                best_score[1] = p[1];
                best_score[2] = p[2];
                case_num = 3;
              }
              // Case 4: current_error > best_error (first else if) 
              else if(cummulative_error > best_error && case_two == false){
                p[index_pos] = p[index_pos] - 3*dp[index_pos];
                case_num = 4;
              } 

              // Updates the values depending the case
              switch (case_num) {
                  case 1: 
                    case_two = false;
                    counter++;
                    break;
                  case 2: 
                    case_two = false;
                    counter++;
                    break;
                  case 3: 
                    counter++;
                    break;
                  case 4:
                    case_two = true;
                    break;
              }
              
              //Reset the values
              cummulative_error = 0.0;
              iterations =0;
              present_error = 0.0;

              // Change coefficient index
              index_pos = counter % 3;
              
              // Reset the simulation
              string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }

            else{
              if(iterations == 0){
                p[index_pos]+= dp[index_pos];
                pid.Init(p[0],p[1],p[2]);

                std::cout << "----------------------------------------------------------------------------------" << std::endl;
                std::cout << " Best score at the moment P : " << best_score[0] << "  I : " << best_score[1] << "  D : " << best_score[2] << std::endl;
                std::cout << " Current coefficients     P : " << p[0] << "  I : " << p[1] << "  D : " << p[2] << std::endl;
                std::cout << " DP array values are      F : " << dp[0] << " S :  " << dp[1] << " T : " << dp[2] << std::endl;
                std::cout << " Position of the index ( 0 = P , 1 = I , 2 = D)" << index_pos << std::endl;
                std::cout << " Counter : " <<counter << std::endl;
              }

              // Updates the Cross Track Error (CTE):
              pid.UpdateError(cte);
              
              // Calculate the total error and assign the value to the steer value. 
              steer_value = pid.TotalError();

              //Control the velocity
              throttle = control_velocity(speed,angle);

              // Store all the errors and update the iterations 
              present_error = fabs(cte);
              cummulative_error += present_error;
              iterations++;
            }
          }

          // twiddle flag false (Try the optimized parameters after the twiddle)
          else{
            // Updates the Cross Track Error (CTE):
              pid.UpdateError(cte);
              
              // Calculate the total error and assign the value to the steer value. 
              steer_value = pid.TotalError();

              //Control the velocity
              throttle = control_velocity(speed,angle);
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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