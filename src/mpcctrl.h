#ifndef __MPCCTRL_H
#define __MPCCTRL_H

#ifdef __cplusplus
extern "C" {
#endif

bool MPC_Init(double pos_x, double vel_x, double theta, double thetadot, double f1, double f2, double f3, double f4, double b1, double b2, double rbar);
bool MPC_Run(double x_in, double in_xdot, double theta, double thetadot, double* u);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __MPCCTRL_H */
