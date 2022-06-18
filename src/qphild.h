#ifndef __QPHILD_H
#define __QPHILD_H

#include "Eigen/Dense"

/******************************************************************************
 * DEFINES
 ******************************************************************************/

#define QPHILD_MAXITER      512     /* 10 * (H.rows() + A.rows()) */
#define QPHILD_TOLER        1e-8

/******************************************************************************
 * FUNCTION
 ******************************************************************************/

/** @brief Solves a quadratic programming problem.
 *    x = qphild(H,f,A,b) attempts to solve the quadratic programming 
    problem:
             min 0.5*x'*H*x + f'*x   subject to:  A*x <= b 

    If this fails, let's use this:
    https://github.com/quadprog/quadprog/blob/master/quadprog/solve.QP.c
*/
bool qphild(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& H,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& f,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& A,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
    Eigen::Matrix<double, Eigen::Dynamic, 1>& x,
    const int maxIter = QPHILD_MAXITER,
    const double tolerance = QPHILD_TOLER);

#endif
