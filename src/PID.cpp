#include "PID.h"
#include <iostream>
#include <cmath>
using namespace std;

/*
* Implements PID control with Twiddle.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p = new double[num_params];
    dp = new double[num_params];
    index = 0;
    // initialize p and dp
    p[0] = Kp; dp[0] = 0.1*Kp;
    p[1] = Ki; dp[1] = 0.1*Ki;
    p[2] = Kd; dp[2] = 0.1*Kd;
    //Note: for some reason, numeric_limits was not being recognized!
    //TODO: use std::numeric_limits to initialize max.
    best_cte = 1000000000.0; // initialize to max.
    twiddle_step = 1; // start with step 1 of twiddle algo
    i_error = 0.0; // initialize integral error component to 0.0
    step = 1; // start steps count from 1
}

void PID::UpdateError(double cte) {
    // update errors
    if (step == 1) prev_cte = cte; // initialize correctly for first step
    p_error = cte; // proportional component
    i_error += cte; // integral component, accumulates over time
    d_error = cte - prev_cte; // differential component, considering delta_t = 1

    std::cout << "Ep: " << p_error
              << " Ei: " << i_error
              << " Ed: " << d_error
              << std::endl;

    // save cte for next iteration
    prev_cte = cte;

    // below logic implements twiddle algo for an asynchronous system, to
    // optimize params
    if (step > 100) {
        cte = pow(cte,2); // squared cte seems to show better results
        // cte = cte;
        // first time setup
        if (std::isnan(best_cte)) {
            best_cte = cte; // initialize best_cte
            twiddle_step = 1; // ensure we go to case 1 in the next step
            std::cout << "initializing best cte = " << best_cte << std::endl;
            return;
        }
        std::cout << "Kp: " << p[0]
                  << " Ki: " << p[1]
                  << " Kd: " << p[2]
                  << std::endl;
        // twiddle logic as explaine by Sebastian in the course
        switch (twiddle_step) {
            case 1: // step 1 - increment
                p[index] += dp[index];
                std::cout << "1) incrementing p[" << index << "]: " << p[index]
                          << std::endl;
                twiddle_step = 2;
                return;
            case 2: // evaluate error from step 1
                if (cte < best_cte) { // if there is an improvement
                    best_cte = cte; // save the new best cte
                    dp[index] *= 1.1; // increment dp
                    std::cout << "2) improvement: best cte: " << best_cte
                              << " dp[" << index << "]: " << dp[index]
                              << std::endl;
                    twiddle_step = 1; // go back to step 1
                } else { // if no improvement
                    p[index] -= 2*dp[index]; // decrement P
                    twiddle_step = 3; // go to step 3
                    std::cout << "2) reduction: p[" << index << "]: " << p[index]
                              << std::endl;
                    return;
                }
                break;
            case 3: // evaluate error from step 2
                if (cte < best_cte) { // if there is improvement
                    best_cte = cte; // save the new best cte
                    dp[index] *= 1.1; // increment dp
                    std::cout << "3) improvement: best cte: " << best_cte
                              << " dp[" << index << "]: " << dp[index]
                              << std::endl;
                } else { // if no improvement
                    p[index] += dp[index]; // reset p
                    dp[index] *= 0.9; // decrement dp
                    std::cout << "3) reduction: p[" << index << "]: " << p[index]
                              << " dp[" << index << "]: " << dp[index]
                              << std::endl;
                }
                twiddle_step = 1; // go back to step 1 in either case
                break;
            default: // unknown case
                return;
        }
        index = (index + 1) % num_params; // cycle through all params
    }
    step++;
}

double PID::TotalError() {
    // total error from PID components.
    return -p[0]*p_error - p[1]*i_error - p[2]*d_error;
}

