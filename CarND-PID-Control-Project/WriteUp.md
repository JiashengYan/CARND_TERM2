# PID Control Project Writeup

In this project a PID controller was implemented in order to steer a vehicle and make it follow the reference line. All three parameters were 
optimized by the Twiddle algorithm.


---

## PID Controller
* P controller: acts against current Error, the higher the value, the quicker the error reduces, too large may causes Oscillations.
* I controller: acts against constant errors or systematic bias, its effect accumulates with time.
* D controller: acts against fast changes of system, increase stability and decrease Oscillation and overshoot.

## Twiddle algorithm
* For every parameter, the algorithm first try to increase its value, if the cost funtion is reduced by this change, then keep it and 
increase the amplitude of the change by for example 1.1. If the result is worse, then try to modify the parameter in the reverse direction, and let 
the PID controller run again. If the result is improved, increase the amplitude, if not then reduce the amplitude and keep looping the above steps until the amplitude
 of parameter changes is under a predefined threshold.
* In order to optimize all three parameters, the twiddle algorithm move to next parameter whenever an improvement appears or the direction is changed. So that the algorithm
 won't keep looping on one single parameter.
* There is one shortcomming of the original twiddle algorithm, whenever a parameter is successfully optimized in the negative direction, at the next time when the algorithm come back
 to this parameter, the original twiddle algorithm will still first try to increase its value, this make no scense and can be very inefficient as each run in the simulator take about 2 minutes.
 I therefore modified the algorithm by simply allowing the dp parameter to be negative.
* In order to let the twiddle algorithm to optimise the PID controller on specific road condition, I set two parameters min_n and max_n to mark the begin and stop of twiddle algorithm, so that the PID controller 
will only be optimised for the simulation inbetween.


## Result
The twiddle process takes a lot of time even if the initial values were choosen quiet well, as for every set of parameter, the twiddled controller need to run once in the simulator. 

| [![IMAGE ALT TEXT HERE](https://i9.ytimg.com/vi/IUmdy1rYuI0/mqdefault.jpg?sqp=CMCYgeQF&rs=AOn4CLD7a0Bji13NwkQA953npqYtbFKYuA&time=1551912117116)](https://www.youtube.com/watch?v=IUmdy1rYuI0&t=81s)  | [![IMAGE ALT TEXT HERE](https://i9.ytimg.com/vi/3ZAEMMwPDlE/mqdefault.jpg?sqp=CMCYgeQF&rs=AOn4CLDthNztTZOhfgfeZt8yWWaOVVncWw&time=1551912254589)](https://www.youtube.com/watch?v=3ZAEMMwPDlE&t=20s) |
|:---:|:---:|
| Twiddle process | Driving with the twiddled PID controller |
