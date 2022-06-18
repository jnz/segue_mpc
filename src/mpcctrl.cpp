/******************************************************************************
 * SYSTEM INCLUDE FILES
 ******************************************************************************/

#include <iostream>

/******************************************************************************
 * PROJECT INCLUDE FILES
 ******************************************************************************/

#include "cl1norm.h"
#include "mpcgain.h"
#include "mpcctrl.h"
#include "qphild.h"
#include "Eigen/Dense"

/******************************************************************************
 * DEFINES
 ******************************************************************************/

#define USE_EXT_STATE
#define USE_CL1NORM

/** Number of MPC prediction epochs */
#define N                     50
#define Nc                    25
#define STATE_LEN             4
#define Y_LEN                 2
#define DT_SEC                (1.0/50.0)
#define MODEL_UMAX            1.0
#define MODEL_UMIN           -1.0

#define CL1NORM_INPUT_WEIGHT  2.2

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/******************************************************************************
 * GLOBAL DATA DEFINITIONS
 ******************************************************************************/

/******************************************************************************
 * LOCAL DATA DEFINITIONS
 ******************************************************************************/

/** State x: position, velocity, theta, thetadot */
static Eigen::Matrix<double, STATE_LEN, 1> x;
/** State x from previous epoch */
static Eigen::Matrix<double, STATE_LEN, 1> x_prev;

static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Phi; /* <(N*Y_LEN) x Nc> */
static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> F; /* <(N*Y_LEN) x (STATE_LEN+YLEN)> matrix: augmented state model */
static Eigen::Matrix<double, N*Y_LEN, 1> Rs; /**< reference / setpoint vector */
static Eigen::Matrix<double, Y_LEN, STATE_LEN> Cp; /**< map state to setpoint: y = Rs = Cp*x */

static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HU; /**< <Nc*2 x Nc> control input constraint design matrix */
static Eigen::Matrix<double, Eigen::Dynamic, 1> ULIM; /**< <Nc*2, 1> control input constraints */

static Eigen::Matrix<double, Nc, Nc> H; /**< <NcxNc> cache result of = (Phi.transpose() * Phi + Rbar); */

#ifdef USE_CL1NORM
static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Astar; /* <N*Y_LEN + Nc x Nc> */
static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> lstar; /* <N*Y_LEN + Nc x 1> */
#endif

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************/

/******************************************************************************
 * LOCAL INLINE FUNCTIONS AND FUNCTION MACROS
 ******************************************************************************/

/******************************************************************************
 * FUNCTION BODIES
 ******************************************************************************/

/** @brief Convert a cont. system to a discrete system. */
static void c2dm(const Eigen::Matrix<double, STATE_LEN, STATE_LEN>& A,
                 const Eigen::Matrix<double, STATE_LEN, 1>& B,
                 Eigen::Matrix<double, STATE_LEN, STATE_LEN>& Ap /*output*/,
                 Eigen::Matrix<double, STATE_LEN, 1>& Bp /*output*/,
                 const double dt_sec)
{
    Ap.setIdentity();
    Ap += A*dt_sec;
    Bp = B*dt_sec;
}

bool MPC_Init(double pos_x, double vel_x, double theta, double thetadot, double f1, double f2, double f3, double f4, double b1, double b2, double rbar)
{
    x << pos_x, vel_x, theta, thetadot;
    x_prev = x;

    Eigen::Matrix<double, STATE_LEN, STATE_LEN> A;
    Eigen::Matrix<double, STATE_LEN, 1> B;
    A << 0.0000,  1.0000, 0.0000, 0.0000,
         0.0000,  f1,     f2,     0.0000,
         0.0000,  0.0000, 0.0000, 1.0000,
         0.0000,  f3,     f4,     0.0000;
    B << 0, b1, 0, b2;

    Eigen::Matrix<double, STATE_LEN, STATE_LEN> Ap;
    Eigen::Matrix<double, STATE_LEN, 1> Bp;
    /* Track velocity and angle theta */
    Cp << 0,1,0,0, 0,0,1,0; // y = C*x
    c2dm(A, B, Ap, Bp, DT_SEC);

    Rs.setZero(); /* set zero as reference trajectory */

    HU.resize(Nc*2, Nc);
    HU.setZero();
    HU.block<Nc, Nc>(0, 0).triangularView<Eigen::Lower>().setOnes();
    HU.block<Nc, Nc>(Nc, 0).triangularView<Eigen::Lower>().setOnes();
    HU.block<Nc, Nc>(Nc, 0) *= -1.0;
    ULIM.resize(Nc*2, 1);

    // std::cout << "Ap = " << std::endl << Ap << std::endl << std::endl;
    // std::cout << "Bp = " << std::endl << Bp << std::endl << std::endl;
    // std::cout << "Cp = " << std::endl << Cp << std::endl << std::endl;

#ifdef USE_EXT_STATE
    mpcgainEx(Ap, Bp, Cp, Nc, N, /*out:*/Phi, /*out:*/F);
    #ifdef USE_CL1NORM
    Astar.resize(N*Y_LEN + Nc, Nc);
    Astar.block<Nc, Nc>(N*Y_LEN, 0).setIdentity();
    Astar *= CL1NORM_INPUT_WEIGHT; /* CL1 NORM TUNING PARAMETER SIMILAR TO RBAR */
    Astar.block<N*Y_LEN, Nc>(0, 0) = Phi;
    lstar.resize(N*Y_LEN + Nc, 1);
    lstar.setZero();

    std::cout << "CL1NORM active. Control weight: " << CL1NORM_INPUT_WEIGHT << " Astar = " << std::endl << Astar << std::endl << std::endl;
#else
    std::cout << "Extended state model active." << std::endl << std::endl;
    #endif
#else
    mpcgain(Ap, Bp, Cp, Nc, N, /*out:*/Phi, /*out:*/F);

    ULIM.setOnes();
    ULIM.block<Nc, 1>(0, 0) *= MODEL_UMAX;
    ULIM.block<Nc, 1>(Nc, 0) *= -MODEL_UMIN;
#endif

    Eigen::Matrix<double, Nc, Nc> Rbar;
    Rbar.setIdentity();
    Rbar *= rbar;
    H = (Phi.transpose()*Phi + Rbar);

    return true;
}

/**
 * @param[in] theta Attitude angle in rad
 * @param[in] theta Angular rate in rad/s
 * @param[out] u Control output variable
 */
bool MPC_Run(double x_in, double in_xdot, double theta, double thetadot, double* u)
{
#ifdef USE_EXT_STATE
    Eigen::Matrix<double, Eigen::Dynamic, 1> DU; /* output result */

    ULIM.setOnes();
    ULIM.block<Nc, 1>(0, 0) *= MODEL_UMAX - *u;
    ULIM.block<Nc, 1>(Nc, 0) *= -MODEL_UMIN + *u;

    x << x_in, in_xdot, theta, thetadot;
    const Eigen::Matrix<double, STATE_LEN, 1> dx = x - x_prev;
    x_prev = x;

    Eigen::Matrix<double, STATE_LEN + Y_LEN, 1> x_e;
    x_e.block<STATE_LEN, 1>(0, 0) = dx;
    x_e.block<Y_LEN, 1>(STATE_LEN, 0) = Cp * x; /* y = Cp * x */

  #ifdef USE_CL1NORM
    lstar.block<N*Y_LEN, 1>(0,0) = Rs - F * x_e;
    const cl1_result_t result =
        cl1_double(Astar, lstar, DU, NULL, NULL, &HU, &ULIM);
    const bool success = (result == CL1_OPT_SOL_FOUND);
  #else
    const Eigen::Matrix<double, Nc, 1> f = -Phi.transpose() * (Rs - F * x_e);
    bool success = qphild(H, f, HU, ULIM, DU);
  #endif

    if (success)
    {
        *u = *u + DU(0);
    }
    return success;
#else
    Eigen::Matrix<double, Eigen::Dynamic, 1> U; /* output result */
    x << x_in, in_xdot, theta, thetadot;
    const Eigen::Matrix<double, Nc, 1> f = -Phi.transpose() * (Rs - F * x);
    const int maxIter = 10 * (H.rows() + HU.rows());
    bool success = qphild(H, f, HU, ULIM, U, maxIter, QPHILD_TOLER);
    if (success)
        *u = U(0);
    return success;
#endif
}

