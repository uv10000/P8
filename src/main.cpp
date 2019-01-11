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
  pid.Init(0.12,0.0027*2,2.3);  // 0.05 // (0.04,0.0001,0.1)//(0.05,0.00005,0.2)//}
  //(0.07,0.00005,0.3); (0.08,0.00003,0.5); (0.1,0.00002,0.5); (0.1,0.00002,0.7)
  //(0.2,0.0005,4); (0.8,0.0005,8);
  //(1.2,0.0005,16); quite ok but jerky
  //(2.5,0.002,15);   better but still jerky
  //(0.8,0.0005,5); somewhat better but not satisfactory
  // (0.8,0.0004,8); best so far, may be a starting point for auto-tuning
  //  pid.Init(0.1,0.0004,1); better 
  //(0.05,0.0004,1); quite ok 
  //(0.11,0.005,0.0); also ok but not quite

  //(0.15,0.001,0.45); acceptable with anti windup = 10
  // quite ok wit antiwindup 100 (0.1,0.0007,0.8); 
  //(0.1,0.0014,0.8); even better
  // (0.1,0.002,1.15); a bit jerky but staying away from the curbs
  // this can be submitted (0.11,0.0023,1.5);
  // (0.12,0.0027*2,2.3) terribly kinky but staying away from trouble, submittable


  int counter=0;
  double p[3] = {pid.Kp, pid.Ki, pid.Kd};
  double dp[3] = {0.2, 0.0002, 2};// {1,1,1};
  double best_err= 1e10;
  double err_plus[3]= {0, 0, 0};
  double err_minus[3]= {0, 0, 0};
  double old_err=0;
  int myindex=0;


  h.onMessage([&pid, &counter,&p, &dp,&best_err,&err_plus,&err_minus,&old_err,&myindex](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " averaged sq error: " << pid.TotalError()<< std::endl;
           std::cout << "I Error: " << pid.i_error << std::endl;
          


          //double steer_value_deg = 10.0;
          //steer_value =steer_value_deg*3.14/180.0;
          
          
          /*
          counter=(counter+1)%500;
          if (counter==0) {
            // run another time
            double err = pid.TotalError();
            std::cout << "TotalError: " << err  << std::flush << std::endl;
            while (dp[0]+dp[1]+dp[2] >0.00){
              if(myindex ==0) {
                old_err=err; //remember error at (p1,p2,p3)
                p[0] = p[0] + dp[0]; // prepare for next step
                myindex = (myindex +1)%8;
              }
              else if(myindex ==1) {
                err_plus[0]=err;//compute error at (p1+dp1,p2,p3)
                p[0] = p[0] - dp[0]; // go back
                p[1] = p[1] + dp[1]; // prepare for next step
                myindex = (myindex +1)%8;
              }
              else if(myindex ==2) {
                err_plus[1]=err; //compute error at (p1,p2+dp2,p3)
                p[1] = p[1] - dp[1]; // go back
                p[2] = p[2] + dp[2]; // prepare for next step
                myindex = (myindex +1)%8;
              }
              else if(myindex ==3) {
                err_plus[2]=err; //compute error at (p1,p2,p3+dp3)
                p[2] = p[2] - dp[2]; // go back
                p[0] = p[0] - dp[0]; // prepare for next step
                myindex = (myindex +1)%8;
              }
              else if(myindex ==4) {
                err_minus[0]=err; //compute error at (p1-dp1,p2,p3)
                p[0] = p[0] + dp[0]; // go back
                p[1] = p[1] - dp[1]; // prepare for next step
                myindex = (myindex +1)%8;
              }
              else if(myindex ==5) {
                err_minus[1]=err; //compute error at (p1,p2-dp2,p3)
                p[1] = p[1] + dp[1]; // go back
                p[2] = p[2] - dp[2]; // prepare for next step
                myindex = (myindex +1)%8;
              }
              else if(myindex ==6) {
                err_minus[2]=err;  //compute error at (p1,p2,p3-dp3)
                p[2] = p[2] + dp[2]; // go back
                myindex = (myindex +1)%8;
              }
              else if(myindex ==7) {
                // adapt (p1,p2,p3) and (dp1,dp2,dp3) accordingly
                // three cases:
                // a) right value smaller than middle value -> increase pi, increase dpi
                // b) left value smaller than middle value -> decrease pi, increase dpi
                // c) both values larger than middle value -> keep pi, decrease dpi
                for(int i=0;i<3;i++){
                  if(err_plus[i] < old_err*0.99){
                    p[i]+=dp[i]; dp[i]*=1.1;best_err =err_plus[i];
                  }
                  else if(err_minus[i] < old_err*0.99){
                    p[i]-=dp[i]; dp[i]*=1.1; best_err =err_minus[i];
                  }
                  else{
                    dp[i]*=0.95;
                  }
                  std::cout << "pid.Kp: " << pid.Kp  << "  pid.Ki: " << pid.Ki  << "  pid.Kd : " << pid.Kd  << std::endl;
                  std::cout << "dp[0]: " << dp[0]  << "  dp[1]: " << dp[1]  << "  dp[2]: " << dp[2] << std::endl;
                  std::cout << "best_err: " << best_err  << std::flush << std::endl;
                  std::cout << "old_err: " << old_err  << std::flush << std::endl;

                  pid.Kp=p[0];pid.Ki=p[1];pid.Kd=p[2];
                }
                myindex = (myindex +1)%8;  // prepare to move to s
                // next state after another 100 steps 
              }  
            }
            pid.sum_of_squared_errors =0.0; //start afresh
          }
          */
          

          pid.UpdateError(cte);
          steer_value = -pid.p_error * pid.Kp  - pid.i_error * pid.Ki - pid.d_error * pid.Kd;
          
          double  max_steering_angle = 1.0;
          if (steer_value > max_steering_angle){
            steer_value = max_steering_angle;
          }
          if (steer_value < -max_steering_angle) {
            steer_value = -max_steering_angle;
          }

         
          //speed=2.0;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;//0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // ULI std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else {
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
