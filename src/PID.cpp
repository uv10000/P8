#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  p_error=0.0;
  i_error=0.0;
  d_error=0.0;
  sum_of_squared_errors=0.0;
  prev_cte=0;

}

PID::~PID() {}

void PID::Init(double Kpx, double Kix, double Kdx) {

  Kp=Kpx;
  Ki=Kix;
  Kd=Kdx;

}

void PID::UpdateError(double cte) {
    /*diff_cte = cte - prev_cte
        int_cte += cte
        prev_cte = cte
        steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
        */
    p_error =  cte;
    i_error +=  cte;
    double antiwindup=100.0;
    i_error= fmax(-antiwindup,i_error); i_error=fmin(antiwindup,i_error); // saturate to one
    d_error = cte -prev_cte;
    sum_of_squared_errors = cte*cte + sum_of_squared_errors*0.95;
    prev_cte=cte;

}

double PID::TotalError() {
    return sqrt(sum_of_squared_errors);
}

void PID::Twiddlestep(double *p, double *dp, double &best_err, double err){

std::cout << "P[0]" << p[0] << std::endl;
}
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