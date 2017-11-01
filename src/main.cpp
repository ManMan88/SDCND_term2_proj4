#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <chrono>

// define saturation values
#define SAT_MAX 1.0
#define SAT_MIN -1.0

// define max & min velocity
#define VEL_MAX 100
#define VEL_MIN 10

// define factor parameters to determine speed
#define CTE_MAX 7
#define STEER_MAX 0.8
#define ANGLE_MAX 18

// speed filter
#define ALPHA 0.2


// for convenience
using json = nlohmann::json;
using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/*
** HELPER FUNCTION
** Determine the speed target based on the cte, angle steer_value and last speed
*/
double determine_speed(const double cte, const double angle, const double steer_value, const double speed) {
  // factor the
  double factor = 1 - fabs(cte/CTE_MAX);
  factor *= (1 - fabs(angle/ANGLE_MAX));
  factor *= (1 - fabs(steer_value/STEER_MAX));
  // set the target speed
  double target_speed = VEL_MAX * factor;

  // apply simple filter
  target_speed = target_speed*ALPHA + speed*(1-ALPHA);

  // check that the speed is not too small
  if (target_speed < VEL_MIN)
    target_speed = VEL_MIN;

  return target_speed;
}

// define timing variables
high_resolution_clock::time_point t_last;
high_resolution_clock::time_point t_new;
duration<double> delta_t;


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

int main()
{
  uWS::Hub h;

  PID vel_pid;
  vel_pid.Init(0.04,0.02,0.0005);

  PID steer_pid;
  steer_pid.Init(0.0822805,0.0571301,0.00442457);
  t_last = high_resolution_clock::now();

  h.onMessage([&steer_pid,&vel_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // compute delta time
          t_new = high_resolution_clock::now();
          delta_t = duration_cast<duration<double>>(t_new - t_last);
          double dt = delta_t.count();

          // acquire data
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // update errors
          steer_pid.UpdateError(cte,dt);
          // compute control signal
          double steer_value = steer_pid.TotalError(SAT_MAX,SAT_MIN);

          //determine desired velocity
          double target_speed = determine_speed(cte,angle,steer_value,speed);

          // update velocity error
          vel_pid.UpdateError(speed-target_speed,dt);
          // compute control signal
          double throttle_value = vel_pid.TotalError(SAT_MAX,SAT_MIN);

          //DEBUG
          //std::cout << "target_speed is: " << target_speed << " speed error is: " << speed-target_speed << " throttle is: " << throttle_value << std::endl;

          // update last sample time
          t_last = t_new;

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Angle is: " << angle <<" delta t is: " << dt << std::endl << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
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
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
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
