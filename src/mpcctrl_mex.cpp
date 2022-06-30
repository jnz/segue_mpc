#ifdef MATLAB_MEX_FILE

/*
 * Compile in MATLAB as .mex file with:
 *      mex -O mpcctrl_mex.cpp mpcctrl.cpp mpcgain.cpp qphild.cpp -DCL1NORM_NO_MEX cl1norm.cpp -I./
 */

#include "mex.h"
#include <math.h> /* fabs() */
#include <stdlib.h> /* abs() */
#include "mpcctrl.h"

/* The gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const int min_input = 3;

    if(nrhs != min_input) {
        mexErrMsgIdAndTxt(
        "JanZwiener:mpc",
        "2-Wheel Robot Model Predictive Control (MPC).\n"
        "Usage: [u] = mpcctrl(x, x_prev, u);\n"
        "");
    }

    if(nlhs != 1) {
        mexErrMsgIdAndTxt("JanZwiener:mpc",
                          "Output required.");
    }

    /* make sure the input arguments are real */
    for (int i=0;i<nrhs;i++)
    {
        if( !mxIsDouble(prhs[i]) || mxIsComplex(prhs[i]) )
        {
            mexErrMsgIdAndTxt("JanZwiener:cl1norm:notReal",
                              "Input must be real.");
        }
    }

    const mxArray* mX     = prhs[0];
    const mxArray* mXprev = prhs[1];
    const mxArray* mUin   = prhs[2];

    const int n = (int)mxGetM(mX);
    const int cols = (int)mxGetN(mX);
    const int nprev = (int)mxGetM(mX);
    const int colsprev = (int)mxGetN(mX);
    const int nu = (int)mxGetM(mUin);
    const int mu = (int)mxGetN(mUin);

    if (cols != 1 || n != 4)
    {
        mexErrMsgIdAndTxt("JanZwiener:mpc",
                          "X is not a row vector with length 4");
    }
    if (cols != colsprev || n != nprev)
    {
        mexErrMsgIdAndTxt("JanZwiener:mpc",
                          "X and Xprev dimension mismatch");
    }
    if (nu != 1 || mu != 1)
    {
        mexErrMsgIdAndTxt("JanZwiener:mpc",
                          "u is not scalar");
    }

    const double* xx = mxGetPr(mX);
    const double* xp = mxGetPr(mXprev);
    const double* up = mxGetPr(mUin);

    const double pos_x         = xx[0];
    const double vel_x         = xx[1];
    const double theta         = xx[2];
    const double thetadot      = xx[3];

    const double pos_x_prev    = xp[0];
    const double vel_x_prev    = xp[1];
    const double theta_prev    = xp[2];
    const double thetadot_prev = xp[3];

    double f1 = -7.54;
    double f2 = 0.03;
    double f3 = 0;
    double f4 = 30;
    double b1 = 5.73;
    double b2 = -200;
    double rbar = 1.5;

    if(!MPC_Init(pos_x_prev, vel_x_prev, theta_prev, thetadot_prev, f1, f2, f3, f4, b1, b2, rbar))
    {
        mexErrMsgIdAndTxt("JanZwiener:mpc",
                          "MPC_Init failed");
    }

    double u = up[0];
    if(!MPC_Run(pos_x, vel_x, theta, thetadot, &u))
    {
        mexErrMsgIdAndTxt("JanZwiener:mpc",
                          "MPC_Run failed");
    }

    mxArray* mU = mxCreateDoubleMatrix(1, 1, mxREAL);
    double* pU  = mxGetPr(mU);
    *pU = u;
    plhs[0] = mU;
}

#endif /* MATLAB_MEX_FILE */
