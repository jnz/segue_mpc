#ifndef __MPCCTRL_H
#define __MPCCTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MPC.
 *
 * @param[in] pos_x Initial robot 1-D horizontal position x in meters (forward/backwards).
 * @param[in] vel_x Initial robot 1-D horizontal velocity x in m/s e.g. from wheel odometry.
 * @param[in] theta Initial robot angle (in rad). 0 = upright position.
 * @param[in] thetadot Initial robot angular velocity along wheel axis (in rad/s).
 * @param[in] f1 System model parameter (see below)
 * @param[in] f2 System model parameter (see below)
 * @param[in] f3 System model parameter (see below)
 * @param[in] f4 System model parameter (see below)
 * @param[in] b1 System model parameter (see below)
 * @param[in] b2 System model parameter (see below)
 * @return true if successful.
 *
 * STATE SPACE MODEL
 * -----------------
 *
 *    4x1 state vector x of two wheel robot:
 *
 *    x = [ position(m)  velocity(m/s) theta(rad)  thetadot(rad/s) ]
 *
 *    u = 1x1 Wheel motor control input variable between 1.0 (max. forward
 *          speed) and -1.0 (max. backward speed)
 *
 *
 *    A = [0      1              0                0;
 *         0      f1             f2               0;
 *         0      0              0                1;
 *         0      f3             f4               0];
 *
 *    B = [     0;
 *              b1;
 *              0;
 *              b2];
 *
 *    xdot(k) = A*x(k) + B*u(k)
 */
bool MPC_Init(double pos_x, double vel_x, double theta, double thetadot, double f1, double f2, double f3, double f4, double b1, double b2, double rbar);

void MPC_SetThetaRef(double theta);

/**
 * @brief Calculate next control input command u.
 *
 * @param[in] pos_x Current robot 1-D horizontal position x in meters (forward/backwards).
 * @param[in] vel_x Current robot 1-D horizontal velocity x in m/s e.g. from wheel odometry.
 * @param[in] theta Current robot angle (in rad). 0 = upright position.
 * @param[in] thetadot Current robot angular velocity along wheel axis (in rad/s).
 * @param[out] u 1x1 Result: control input command u (between -1.0 and 1.0).
 * @return true if result in u can be used.
 * */
bool MPC_Run(double x_in, double in_xdot, double theta, double thetadot, double* u /*output*/);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __MPCCTRL_H */
