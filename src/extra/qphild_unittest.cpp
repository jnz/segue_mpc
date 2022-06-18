/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <assert.h>
#include <iostream>

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

#include "../qphild.h"

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/******************************************************************************
 * GLOBAL DATA DEFINITIONS
 ******************************************************************************/

/******************************************************************************
 * LOCAL DATA DEFINITIONS
 ******************************************************************************/

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************/

/******************************************************************************
 * LOCAL INLINE FUNCTIONS AND FUNCTION MACROS
 ******************************************************************************/

/******************************************************************************
 * FUNCTION BODIES
 ******************************************************************************/

static void qphild_test_simple(void)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H(2,2);
    Eigen::Matrix<double, Eigen::Dynamic, 1> f(2,1);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(3,2);
    Eigen::Matrix<double, Eigen::Dynamic, 1> b(3,1);

    H << 1.0, -1.0, -1.0, 2.0;
    f << -2, -6;
    A << 1, 1, -1, 2, 2, 1;
    b << 2, 2, 3;

    Eigen::Matrix<double, Eigen::Dynamic, 1> result;
    bool success = qphild(H, f, A, b, result);

    assert(success);
    assert(fabs(result(0) - 0.666666666667) < 1e-4 &&
        fabs(result(1) - 1.333333333333) < 1e-4);
}

static void qphild_least_squares(void)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M(3,3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> b(3,1);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(3,3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> s(3,1);

    M << 1, 2, 0, -8, 3, 2, 0, 1, 1;
    b << 3, 2, 3;
    A << 1, 2, 1, 2, 0, 1, -1, 2, -1;
    s << 3, 2, -2;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> q;
    P = M.transpose() * M;
    q = -M.transpose() * b;

    Eigen::Matrix<double, Eigen::Dynamic, 1> result;
    bool success = qphild(P, q, A, s, result);

    assert(success);
    assert(fabs(result(0) -   0.12997347) < 1e-4 &&
           fabs(result(1) -  -0.06498674) < 1e-4 &&
           fabs(result(2) -   1.74005305) < 1e-4);
}

void qphild_test(void)
{
    qphild_test_simple();
    qphild_least_squares();
}
