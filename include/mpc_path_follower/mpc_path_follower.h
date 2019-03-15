/*********************************************************************
 * @file:   mpc_path_follower.h
 * @author: Lianchuan Zhang
 * @date:   2019.01.16
 * @version:1.0.1
 * @brief:  using mpc method to do path tracking
 *********************************************************************/
#ifndef _MPC_PATH_FOLLOWER_H_
#define _MPC_PATH_FOLLOWER_H_

#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;
size_t N = 10; //timesteps
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
double dt = 0.1; //frequency
double ref_v = 70; //refence_velocity
// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start;
size_t y_start;
size_t psi_start;
size_t v_start;
size_t cte_start;
size_t epsi_start;
size_t delta_start;// steering angle
size_t a_start;// acceleration

//class that computes objective and constraints
class FG_eval{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    Eigen::VectorXd coeffs;
    // Coefficients of the fitted polynomial.
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs;}
    void operator()(ADvector& fg, const ADvector& vars){
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        // TODO: Define the cost related the reference state and
        // any anything you think may be beneficial.
        for (int t = 0; t < N; t++) {
          fg[0] += 3000*CppAD::pow(vars[cte_start + t], 2);
          fg[0] += 3000*CppAD::pow(vars[epsi_start + t], 2);
          fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
          fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
          fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
          // try adding penalty for speed + steer
          fg[0] += 700*CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
        }
        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
          fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
          fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }
        // Setup Constraints
        // Initial constraints
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int t = 1; t < N; t++) {
          // The state at time t+1 .
          AD<double> x1 = vars[x_start + t];
          AD<double> y1 = vars[y_start + t];
          AD<double> psi1 = vars[psi_start + t];
          AD<double> v1 = vars[v_start + t];
          AD<double> cte1 = vars[cte_start + t];
          AD<double> epsi1 = vars[epsi_start + t];

          // The state at time t.
          AD<double> x0 = vars[x_start + t - 1];
          AD<double> y0 = vars[y_start + t - 1];
          AD<double> psi0 = vars[psi_start + t - 1];
          AD<double> v0 = vars[v_start + t - 1];
          AD<double> cte0 = vars[cte_start + t - 1];
          AD<double> epsi0 = vars[epsi_start + t - 1];

          // Only consider the actuation at time t.
          AD<double> delta0 = vars[delta_start + t - 1];
          AD<double> a0 = vars[a_start + t - 1];

          if (t > 1) {   // use previous actuations (to account for latency)
            a0 = vars[a_start + t - 2];
            delta0 = vars[delta_start + t - 2];
          }

          AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
          AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

          // Here's `x` to get you started.
          // The idea here is to constraint this value to be 0.
          //
          // Recall the equations for the model:
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          //!!!!!!!!!!there some changes on psi and epsi
          fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
          fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
          fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);//这个地方可能是符号的问题
          fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
          fg[1 + cte_start + t] =
              cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
          fg[1 + epsi_start + t] =
              epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
        }
    }
};

//class mpc path follower

class MPC_Path_Follower{
public:
    typedef CPPAD_TESTVECTOR(double) Dvector;
    MPC_Path_Follower() = default;
    void initialize();
    std::vector<double> solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    ~MPC_Path_Follower() = default;

private:
    size_t i;
    //state variables and size
    double x;
    double y;
    double psi;
    double v;
    double cte;
    double epsi;
    size_t n_vars;
    size_t n_constraints;
    std::string options;
    bool ok;
    std::vector<double> result;
};
#endif
