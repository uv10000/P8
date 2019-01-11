#include "PID.h"
#include <math.h>

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
    d_error = cte -prev_cte;
    sum_of_squared_errors = cte*cte + sum_of_squared_errors*0.95;
    prev_cte=cte;

}

double PID::TotalError() {
    return sqrt(sum_of_squared_errors);
}

