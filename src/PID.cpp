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
    double antiwindup=100.0;  // another anti-windup for limiting I-Integral 
    i_error= fmax(-antiwindup,i_error); i_error=fmin(antiwindup,i_error); 
    // saturate to "antiwindup"
    d_error = cte -prev_cte;
    sum_of_squared_errors = cte*cte + sum_of_squared_errors*0.95; // moving average!
    prev_cte=cte;
}

double PID::TotalError() {
    return sqrt(sum_of_squared_errors);
}

void PID::Twiddlestep(double *p, double *dp, double &best_err, double err){
/*
* Unfinished, "hide" the implementation and lots of global variable
* of the online "Twiddle" from the main. see writeup
*/
std::cout << "P[0]" << p[0] << std::endl;
}