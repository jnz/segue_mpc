/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <assert.h>

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

#include "qphild.h"

/******************************************************************************
 * FUNCTION BODIES
 ******************************************************************************/

bool qphild(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& H,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& f,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& A,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
    Eigen::Matrix<double, Eigen::Dynamic, 1>& x,
    const int maxIter,
    const double tolerance)
{
    assert(H.rows() == f.rows());
    assert(A.rows() == b.rows());
    assert(f.rows() == A.cols());

    const int n1 = (int)A.rows();
    x = -H.householderQr().solve(f);

    int i;
    for (i=0;i<n1;i++)
    {
        if (A.row(i) * x > b(i))
            break;
    }
    if (i == n1)
    {
        /* no constraint violated, directly use least squares solution */
        return true;
    }

    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HAT =
        H.householderQr().solve(A.transpose());
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P = A * HAT;
    const Eigen::Matrix<double, Eigen::Dynamic, 1> d =
        A*(H.householderQr().solve(f)) + b; /* FIXME use previous cached result */

    const int n = (int)d.rows();
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_ini(n, 1);
    x_ini.setZero();
    Eigen::Matrix<double, Eigen::Dynamic, 1> lambda = x_ini;
    Eigen::Matrix<double, Eigen::Dynamic, 1> lambda_p;
    Eigen::Matrix<double, 1, 1> wx;

    int km;
    for (km = 0; km < maxIter; km++)
    {
        lambda_p = lambda;
        for (int i = 0; i < n; i++)
        {
            wx = P.row(i) * lambda;
            const double w = wx(0,0) - P(i, i) * lambda(i, 0) + d(i, 0);
            const double la = -w / P(i, i);
            lambda(i, 0) = la > 0.0 ? la : 0.0;
        }
        if ((lambda - lambda_p).squaredNorm() < QPHILD_TOLER)
        {
            break;
        }
    }
    x -= HAT * lambda;

    return true; /* qphild result can be used even if km >= maxIter */
}

