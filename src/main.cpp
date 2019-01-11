#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

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

int main()
{
  uWS::Hub h;
  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.8,0.0004,8);  // 0.05 // (0.04,0.0001,0.1)//(0.05,0.00005,0.2)//
  //(0.07,0.00005,0.3); (0.08,0.00003,0.5); (0.1,0.00002,0.5); (0.1,0.00002,0.7)
  //(0.2,0.0005,4); (0.8,0.0005,8);
  //(1.2,0.0005,16); quite ok but jerky
  //(2.5,0.002,15);   better but still jerky
  //(0.8,0.0005,5); somewhat better but not satisfactory
  // (0.8,0.0004,8); best so far, may be a starting point for auto-tuning

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " max_error: " << pid.TotalError()<< std::endl;

          /*
          # Make this tolerance bigger if you are timing out!
          def twiddle(tol=0.2): 
          # Don't forget to call `make_robot` before every call of `run`!
          p = [0, 0, 0]
          dp = [1, 1, 1]
          robot = make_robot()
          x_trajectory, y_trajectory, best_err = run(robot, p)
          # TODO: twiddle loop here
          x_trajectory, y_trajectory, best_err=run(robot,p)
          while sum(dp) >tol:
          for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err=run(robot,p)
            if err< best_err:
              best_err=err
              dp[i]*=1.1
            else:
              p[i]-=2*dp[i]
              robot = make_robot()
              x_trajectory, y_trajectory, new_err=run(robot,p)
              if new_err< best_err:
                best_err=new_err
                dp[i]*=1.1
              else:
                p[i]+=dp[i]
                dp[i]*=0.9
    
                return p, best_err



          */


          //double steer_value_deg = 10.0;
          //steer_value =steer_value_deg*3.14/180.0;
          

          pid.UpdateError(cte);
          steer_value = -pid.p_error * pid.Kp  - pid.i_error * pid.Ki - pid.d_error * pid.Kd;
          
          double  max_steering_angle = 1.0;
          if (steer_value > max_steering_angle){
            steer_value = max_steering_angle;
          }
          if (steer_value < -max_steering_angle) {
            steer_value = -max_steering_angle;
          }

         
          speed=3.0;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // ULI std::cout << msg << std::endl;
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
