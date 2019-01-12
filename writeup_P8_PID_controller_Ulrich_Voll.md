# **PID Controller** 




**P8 PID Controller Project, Ulrich Voll**

Please refer to my githup repo [P8](https://github.com/uv10000/P8).

The goals / steps of this project according to the [rubric points](https://review.udacity.com/#!/rubrics/1972/view) are the following:


1 Your code should compile.

2 The PID procedure follows what was taught in the lessons.

3 Describe the effect each of the P, I, D components had in your implementation.

4 Describe how the final hyperparameters were chosen.

5 The vehicle must successfully drive a lap around the track.



[//]: # (Image References)

[image1]: ./examples/placeholder.png "Model Visualization"

---------------------

#### 0. Executive Summary   

The solution submitted should fulfil all rubric points but I do not find the result particularly pleasing. See discussion and final section on possible improvements and further steps.

My best sets of parameters (P,I,D) were
-  (0.11,0.0023,1.5)
-  (0.12,0.0054,2.3) 
 
The car can drive safely around the track at some 33 mph for an indefinite number of rounds. It stays away from the curbs at all times. 
However its driving style is unpleasingly jerky/wobbly. 

I did not implement any speed control. I just set the throttle to 0.3, open loop. 





My original plan was to feed a just about acceptable hand-tuned initial set of parameters  to an automatic tuning mechanism implementing an on-line modification of Sebastian's  "Twiddle" search. "Online" standing for  "at run time" in this context. 

I was not able to fully get this online-Twiddle to work, given my limited time resources, but I  will describe the algorithmic ideas and present code fragments below. 



I ended up with a lot of hand tuning, yielding the aforementioned parameter values. 





I suppose not only the mean squared error but also this "jerkyness" of the trajectory should be punished by an appropriate cost function for optimization. (Once one has a working optimization framework at hand ...)

Also and somewhat independently of the "jerky  trajectory" issue, it might be worth punishin fast steering movements. It is conceivable that the car itself drives smoothely although the steering movements themselves are not. However, I think that very good human drivers, when going close to the limits do also steer in a somewhat "near-discontinous" way. 

Still my solution exhibits some discontinuous steering which happens definitely in the wrong places, compared to what to expect from a good human driver. My point is that fast steering movements are not necessarily wrong per se. -> It is hard to find the right cost function for optimization (even if one has a working implementation of the optimization algorithm.)

Remark: Other than a human driver the PID controller is not able to look ahead. I think that (good) human driving is dominated by looking ahead (=open loop control) rather than closed loop control.  Maybe combining PID with the "end-to-end" behaviour from the Behavioural Cloning project could be a step in the right direction.

If a driver reacts to deviations only after they have already happend, he or she is definitely reacting too slowly. The I-part may help to some extend but this is looking behind, not ahead, so the I-part can merely help in extrapolating past behaviour, which can be regarded as  some kind of prediction. 

It appeared to me that smooth trajectories and spatial precision (i.e. staying away from the curbs at all times) are somewhat conflicting goals. But maybe there are better parameters than the ones I found. 

After all, there are only three parameters in a PID controller --  that may not be sufficient to find a single simultaneous match for all needs/requirements. And it this is certainly not enough to mimick complex human driving-behaviour (wheras the CNNs have huge numbers of parameters, arguably far too many, ... )

For example it might be appropriate to choose the parameters dependent on vehicle speed (and potentially many other influencing factors). 






---




#### 1. Your code should compile.

 
Please refer to my githup repo [P8](https://github.com/uv10000/P8), cmake and make should work in the standard way.

I checked with a clean clone, seems to work fine on my local setup. 


-----

#### 2. The PID procedure follows what was taught in the lessons.
See lines 36 ff of PID.cpp
```
 p_error =  cte;
    i_error +=  cte;
    double antiwindup=100.0;  // another anti-windup for limiting I-Integral 
    i_error= fmax(-antiwindup,i_error); i_error=fmin(antiwindup,i_error); 
    // saturate to "antiwindup"
    d_error = cte -prev_cte;
    sum_of_squared_errors = cte*cte + sum_of_squared_errors*0.95; // moving average!
    prev_cte=cte;
```
and line 186 of main.cpp
```
 steer_value = -pid.p_error * pid.Kp  - pid.i_error * pid.Ki - pid.d_error * pid.Kd;
```

Note that I included an anti-windup mechanism for the I error integral and a moving averaging mechanism to prevent the value of the mean squared errors from winding up. 

---------------

#### 3. Describe the effect each of the P, I, D components had in your implementation.

In theory the P part should be for following the road, the D part should prevent overshooting and damping if the P part is chosen aggressively (which may be necessary in order to stay away from curbs ...). And the I part is for stationary accuracy.

However in practice this does not tell the whole story because the overall system consists of the plant (vehicle with limitations, inertia, etc.) plus the controller. 
In simple (linear, time invariant, no saturation or finite actuator velocities, etc.) cases and for simple controllers (P-controller) it is possible to find the right controller parameters algebraically using Laplace transforms. 

Speaking from my personal experience, I find this (i.e. the role of classical linear control theory, to be honest) somewhat overrated in view of many practical applications that are neither linear nor time invariant. All those shiny Laplace transform methods are no longer (strictly) applicable as soon as the slightest non-linearity or time dependeny creeps in.  What is your opinion on this? 

I totally agree with you that parameter tuning by optimization like e.g. by using Twiddle is a far more broadly applicable method, which works well in practice.  

Btw. it is a pity that you removed the  Model Predictive Control module from the course.
Which is - as far as I understand - a method for online optimisation of controller parameters.  

---------------------------

#### 4 Describe how the final hyperparameters were chosen.

Before  I will sketch my incomplete ideas and code fragments for an online version of Twiddle for autotuning below, here comes my "strategy" for manual tuning:

a) Start with I and D parts set to zero

b) Turn up the P value until the system gets instable.

c) Compensate for instability by turning  up the D part, and/or reducing P. Return to b) and iterate

d) Slowly turn up the I part to improve static accuracy, possibly return to b) and reiterate.  

(Plus spend hours of twiddling around by hand in a less than systematic way.)


Here comes the idea for performing twiddle online, that is at-run-time:

In an online setting we cannot restart the system at random during optimisatin, as we did in the quiz.

Idea: 

- State machine with 8 states arranged in a circle. 

- Move from state to state in cylic order.

- In each state certain parameter changes are performed "in the twiddle spirit", details see below. 

- Each time we leave a state the moving-average mean squared error counter is reset. 

- Inbetween states wait for several (e.g. 500) iterations of the simulator for the respective parameter changes to take effect, i.e. until the  moving-average mean squared error is close to it's steady state value. 

- During one round in the circle we collect enough information to perform (discrete) partial derivatives of the cost function, evaluated at the original set of parameter.

- In the final state of the cycle the parameters are updated according to twiddle. 

Remark: Twiddle is a generalization of gradient descent, with variable adaptive step size, isn't it? 

See the following (unfinished) code snippet for further details: 

```
          // this is placed in the main loop i.e. in the body of h.onMessage
          counter=(counter+1)%500;  // wait 500 steps before moving to the next state
          if (counter==0) {  // move to the next state
            double err = pid.TotalError();  // should have converged during 500 steps
            std::cout << "TotalError: " << err  << std::flush << std::endl;
            while (dp[0]+dp[1]+dp[2] >0.00){
              if(myindex ==0) {  // state-machine with 8 states 
                old_err=err; //remember error at (p1,p2,p3)
                p[0] = p[0] + dp[0]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==1) {
                err_plus[0]=err;//compute error at (p1+dp1,p2,p3)
                p[0] = p[0] - dp[0]; // go back
                p[1] = p[1] + dp[1]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==2) {
                err_plus[1]=err; //compute error at (p1,p2+dp2,p3)
                p[1] = p[1] - dp[1]; // go back
                p[2] = p[2] + dp[2]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==3) {
                err_plus[2]=err; //compute error at (p1,p2,p3+dp3)
                p[2] = p[2] - dp[2]; // go back
                p[0] = p[0] - dp[0]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==4) {
                err_minus[0]=err; //compute error at (p1-dp1,p2,p3)
                p[0] = p[0] + dp[0]; // go back
                p[1] = p[1] - dp[1]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==5) {
                err_minus[1]=err; //compute error at (p1,p2-dp2,p3)
                p[1] = p[1] + dp[1]; // go back
                p[2] = p[2] - dp[2]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==6) {
                err_minus[2]=err;  //compute error at (p1,p2,p3-dp3)
                p[2] = p[2] + dp[2]; // go back
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
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
                  pid.Kp=p[0];pid.Ki=p[1];pid.Kd=p[2];
                }
                myindex = (myindex +1)%8;  // jump to the next state after 500 steps
              }  
            }
            pid.sum_of_squared_errors =0.0; //start afresh, reset error counter
          }
```

------------------
#### 5 The vehicle must successfully drive a lap around the track.


It does, but the driving style is rather jerky. See above.

I found it hard to find a set of parameters that simultaneously leads to a "smooth" driving style while staying close to the middle of the road, in particular away from the curbs. 


------------------
#### 6 Further improvements.

- Make the online optimisation (Twiddle) work. I have run out of time but I am confident that the above idea could work out. 

- alternatively find a way for offline optimization (eg recording an interesting part of the track). I expect this to be a lot of fiddly work though.

- Introduce an element of "looking ahead" of some kind, e.g. by using the CNN from the "Behavioural Cloning" project. Or just by providing an "unfair and artificial ad-hoc-hint" derived from our knowledge of the track, justified by the fact that a human driver will be able to look ahead.

- Model Predictive Control ... 
