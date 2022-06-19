#ifndef __QPHILD_H
#define __QPHILD_H

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

#include "Eigen/Dense"

/******************************************************************************
 * DEFINES
 ******************************************************************************/

#define QPHILD_MAXITER      256     /* 10 * (H.rows() + A.rows()) */
#define QPHILD_TOLER        1e-8

/******************************************************************************
 * FUNCTION
 ******************************************************************************/

/** @brief Quadratic Programming (QP) Solver
 *
*     x = qphild(H,f,A,b) attempts to solve the
*     quadratic programming problem in the following form:
*
*            min 0.5*x'*H*x + f'*x 
*
*            subject to:  A*x <= b 
*
*   Result: vector x (length Nc)
*
*   Least squares problems in the form M*x = l can be transformed:
*       H = M'*M
*       f = -M'*l
*
*   @param[in] H const. input matrix (Nc x Nc)
*   @param[in] f const. input vector (Nc x 1)
*   @param[in] A const. constraint matrix for 1 to Nk constraints (Nk x Nc)
*   @param[in] b const. constraint vector A*x <= b (Nk x 1)
*   @param[out] x output solution vector x (Nc x 1)
*   @param[in] maxIter Optional: max. number iterations to satisfy constraints
*   @param[in] tolerance Optional: numeric tolerance
*   @return True if solution can be used
*
*   Hildreth’s Quadratic Programming Procedure
*   "The algorithm will give a compromised, near-optimal solution
*   with constraints if the situation of conflict constraints arises."
*
*   Source (Wang 2009, ISBN 978-1-84882-330-3)
*   "Model Predictive Control System Design and Implementation Using MATLAB"
*/
bool qphild(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& H,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& f,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& A,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
    Eigen::Matrix<double, Eigen::Dynamic, 1>& x,
    const int maxIter = QPHILD_MAXITER,
    const double tolerance = QPHILD_TOLER);

#endif
