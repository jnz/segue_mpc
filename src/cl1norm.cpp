/**
 * @file cl1norm.cpp
 *
 * SIMPLEX LINEAR PROGRAMMING SOLVER
 * =================================
 *
 * MEX Arguments: A, B, C, D, E, F
 *  A*X = B
 *  C*X = D
 *  E*X <= F
 *
 * Return value:
 *  X
 *  RESIDUALS (optional)
 *  INFO (optional)
 *
 * @brief
 * This function uses a modification of the simplex
 * method of linear programming to calculate an l1 solution
 * to a k-by-n system of linear equations
 *             A*X=B
 * subject to l linear equality constraints
 *             C*X=D
 * and m linear inequality constraints
 *             E*X <= F
 *
 * C/C++ code by Jan Zwiener (2015).
 *
 * INFO vector:
 * INFO(1) = return code:
 *                        0- optimal solution found,
 *                        1- no feasible solution to the
 *                           constraints,
 *                        2- calculations terminated
 *                           prematurely due to rounding errors,
 *                        3- maximum number of iterations reached.
 *
 * INFO(2) = minimum sum of absolute values of the residuals.
 * INFO(3) = number of simplex iterations.
 *
 * Based on:
 *     I. Barrodale and F. D. K. Roberts. 1980.
 *     Algorithm 552:
 *     Solution of the Constrained I1 Linear Approximation Problem [F4].
 *     ACM Trans. Math. Softw. 6, 2 (June 1980), 231-235.
 *
 *     FORTRAN code:
 *     552.f -- translated by f2c (version 20100827).
 *
 * Compile in MATLAB as .mex file with:
 *      mex -O cl1norm.cpp
 * @{ */

/******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#ifdef CL1NORM_NO_MEX 
#undef MATLAB_MEX_FILE
#endif

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#else
#include <stdio.h> /* printf */
#include <stdbool.h> /* bool type */
#include "cl1norm.h" /* Eigen interface header */
#endif

#include <math.h> /* fabs() */
#include <stdlib.h> /* abs() */

/******************************************************************************
 * DEFINES
 ******************************************************************************/

#define ENABLE_EIGEN_CPP_INTERFACE
#define DEFAULT_TOLERANCE (2e-11) /* change the MATLAB mexFunction() help text
                                    if you change this */

#define DEFAULT_TOLERANCE_FLOAT (1e-5)

/******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/* some typedefs used in the cl1() function */
#ifdef MATLAB_MEX_FILE
typedef double real;
#else
using namespace Eigen;
#endif

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************/

/******************************************************************************
 * FUNCTION BODIES
 ******************************************************************************/

/* helper function to make sure the array access is not out of bound */
#ifdef NDEBUG
#define DBGCHECK(row, col, rows_total, cols_total, line) (0)
#else
static int DBGCHECK(int row, int col, int rows_total, int cols_total, int line)
{
    row--; col--;
    if (row >= rows_total || col >= cols_total || row < 0 || col < 0)
    {
        printf("Error in line %i: row %i col %i\n", line, row, col);
#ifdef MATLAB_MEX_FILE
        mexErrMsgIdAndTxt("JanZwiener:cl1norm:dimension",
                          "cl1norm internal error.");
#endif
    }
    return 0; /* always return 0 to be transparent to the MAT() macro */
}
#endif

/* helper macro for MATRIX access. This is using column-major access
 * for the MEX interface (MATLAB has Fortran roots).
 * The index is not zero-based! So it can be directly used in the
 * Fortran code.
 * Eigen is also using column-major access. But if you need row-major,
 * there is a macro for that too (see below).
 * DBGCHECK is here to make sure we don't mess something up.
 * DBGCHECK is only active if NDEBUG is not defined. */
#define MAT(M, row, column, rows_total, cols_total) \
    M[(((row)-1) + ((column)-1)*(rows_total)) + \
    DBGCHECK(row, column, rows_total, cols_total, __LINE__)] /* column-major */
/* row-major access: */
#if 0
#define MAT(M, row, column, rows_total, cols_total) \
    M[(((column)-1) + ((row)-1)*(cols_total)) + \
    DBGCHECK(row, column, rows_total, cols_total, __LINE__)]
#endif

/* THIS FUNCTION USES A MODIFICATION OF THE SIMPLEX
   METHOD OF LINEAR PROGRAMMING TO CALCULATE AN L1 SOLUTION
   TO A K BY N SYSTEM OF LINEAR EQUATIONS
               AX=B
   SUBJECT TO L LINEAR EQUALITY CONSTRAINTS
               CX=D
   AND M LINEAR INEQUALITY CONSTRAINTS
               EX.LE.F.
   DESCRIPTION OF PARAMETERS
   K      NUMBER OF ROWS OF THE MATRIX A (K.GE.1).
   L      NUMBER OF ROWS OF THE MATRIX C (L.GE.0).
   M      NUMBER OF ROWS OF THE MATRIX E (M.GE.0).
   N      NUMBER OF COLUMNS OF THE MATRICES A,C,E (N.GE.1).
   KLMD   SET TO AT LEAST K+L+M FOR ADJUSTABLE DIMENSIONS.
   KLM2D  SET TO AT LEAST K+L+M+2 FOR ADJUSTABLE DIMENSIONS.
   NKLMD  SET TO AT LEAST N+K+L+M FOR ADJUSTABLE DIMENSIONS.
   N2D    SET TO AT LEAST N+2 FOR ADJUSTABLE DIMENSIONS
   Q      TWO DIMENSIONAL REAL ARRAY WITH KLM2D ROWS AND
          AT LEAST N2D COLUMNS.
          ON ENTRY THE MATRICES A,C AND E, AND THE VECTORS
          B,D AND F MUST BE STORED IN THE FIRST K+L+M ROWS
          AND N+1 COLUMNS OF Q AS FOLLOWS
               A B
           Q = C D
               E F
          THESE VALUES ARE DESTROYED BY THE SUBROUTINE.
   KODE   A CODE USED ON ENTRY TO, AND EXIT
          FROM, THE SUBROUTINE.
          ON ENTRY, THIS SHOULD NORMALLY BE SET TO 0.
          HOWEVER, IF CERTAIN NONNEGATIVITY CONSTRAINTS
          ARE TO BE INCLUDED IMPLICITLY, RATHER THAN
          EXPLICITLY IN THE CONSTRAINTS EX.LE.F, THEN KODE
          SHOULD BE SET TO 1, AND THE NONNEGATIVITY
          CONSTRAINTS INCLUDED IN THE ARRAYS X AND
          RES (SEE BELOW).
          ON EXIT, KODE HAS ONE OF THE
          FOLLOWING VALUES
               0- OPTIMAL SOLUTION FOUND,
               1- NO FEASIBLE SOLUTION TO THE
                  CONSTRAINTS,
               2- CALCULATIONS TERMINATED
                  PREMATURELY DUE TO ROUNDING ERRORS,
               3- MAXIMUM NUMBER OF ITERATIONS REACHED.
   TOLER  A SMALL POSITIVE TOLERANCE. EMPIRICAL
          EVIDENCE SUGGESTS TOLER = 10**(-D*2/3),
          WHERE D REPRESENTS THE NUMBER OF DECIMAL
          DIGITS OF ACCURACY AVAILABLE. ESSENTIALLY,
          THE SUBROUTINE CANNOT DISTINGUISH BETWEEN ZERO
          AND ANY QUANTITY WHOSE MAGNITUDE DOES NOT EXCEED
          TOLER. IN PARTICULAR, IT WILL NOT PIVOT ON ANY
          NUMBER WHOSE MAGNITUDE DOES NOT EXCEED TOLER.
   ITER   ON ENTRY ITER MUST CONTAIN AN UPPER BOUND ON
          THE MAXIMUM NUMBER OF ITERATIONS ALLOWED.
          A SUGGESTED VALUE IS 10*(K+L+M). ON EXIT ITER
          GIVES THE NUMBER OF SIMPLEX ITERATIONS.
   X      ONE DIMENSIONAL REAL ARRAY OF SIZE AT LEAST N2D.
          ON EXIT THIS ARRAY CONTAINS A
          SOLUTION TO THE L1 PROBLEM. IF KODE=1
          ON ENTRY, THIS ARRAY IS ALSO USED TO INCLUDE
          SIMPLE NONNEGATIVITY CONSTRAINTS ON THE
          VARIABLES. THE VALUES -1, 0, OR 1
          FOR X(J) INDICATE THAT THE J-TH VARIABLE
          IS RESTRICTED TO BE .LE.0, UNRESTRICTED,
          OR .GE.0 RESPECTIVELY.
   RES    ONE DIMENSIONAL REAL ARRAY OF SIZE AT LEAST KLMD.
          ON EXIT THIS CONTAINS THE RESIDUALS B-AX
          IN THE FIRST K COMPONENTS, D-CX IN THE
          NEXT L COMPONENTS (THESE WILL BE =0),AND
          F-EX IN THE NEXT M COMPONENTS. IF KODE=1 ON
          ENTRY, THIS ARRAY IS ALSO USED TO INCLUDE SIMPLE
          NONNEGATIVITY CONSTRAINTS ON THE RESIDUALS
          B-AX. THE VALUES -1, 0, OR 1 FOR RES(I)
          INDICATE THAT THE I-TH RESIDUAL (1.LE.I.LE.K) IS
          RESTRICTED TO BE .LE.0, UNRESTRICTED, OR .GE.0
          RESPECTIVELY.
   ERROR  ON EXIT, THIS GIVES THE MINIMUM SUM OF
          ABSOLUTE VALUES OF THE RESIDUALS.
   CU     A TWO DIMENSIONAL REAL ARRAY WITH TWO ROWS AND
          AT LEAST NKLMD COLUMNS USED FOR WORKSPACE.
   IU     A TWO DIMENSIONAL INTEGER ARRAY WITH TWO ROWS AND
          AT LEAST NKLMD COLUMNS USED FOR WORKSPACE.
   S      INTEGER ARRAY OF SIZE AT LEAST KLMD, USED FOR
          WORKSPACE.
*/
#ifndef MATLAB_MEX_FILE
template <typename real>
#endif
static void cl1(const int *k, const int *l, const int *m,
                const int *n, const int *klmd, const int *klm2d,
                const int *nklmd, const int *n2d, real *q,
                int *kode, const real *toler, int *iter, real *x,
                real *res, real *error, real *cu, int *iu, int *s)
{
    int i, j;
    real z__;
    int n1, n2, ia, ii, kk, in, nk, js;
    real sn, zu, zv;
    int nk1, klm, jmn, nkl, jpn;
    real cuv;
    double sum;
    int klm1, klm2, nkl1, iimn, nklm;
    real xmin, xmax;
    int iout, iineg, maxit;
    real pivot;
    int iphase, kforce;
    real tpivot;
    const int qrows = *klm2d;
    const int qcols = *n2d;

    /* INITIALIZATION. */
    maxit = *iter;
    n1 = *n + 1;
    n2 = *n + 2;
    nk = *n + *k;
    nk1 = nk + 1;
    nkl = nk + *l;
    nkl1 = nkl + 1;
    klm = *k + *l + *m;
    klm1 = klm + 1;
    klm2 = klm + 2;
    nklm = *n + klm;
    kforce = 1;
    *iter = 0;
    js = 1;
    ia = 0;
    /* SET UP LABELS IN Q. */
    for (j = 1; j <= *n; ++j) {
        MAT(q, klm2, j, qrows, qcols) = (real)j;
    }
    for (i = 1; i <= klm; ++i) {
        MAT(q, i, n2, qrows, qcols) = (real)(*n + i);
        if (MAT(q, i, n1, qrows, qcols) >= 0.0) {
            goto L30;
        }
        for (j = 1; j <= n2; ++j) {
            MAT(q, i, j, qrows, qcols) = -MAT(q, i, j, qrows, qcols);
        }
    L30:
        ;
    }
    /* SET UP PHASE 1 COSTS. */
    iphase = 2;
    for (j = 1; j <= nklm; ++j) {
        MAT(cu, 1, j, 2, *nklmd) = (real)0.0;
        MAT(cu, 2, j, 2, *nklmd) = (real)0.0;
        MAT(iu, 1, j, 2, *nklmd) = 0;
        MAT(iu, 2, j, 2, *nklmd) = 0;
    }
    if (*l == 0) {
        goto L60;
    }
    for (j = nk1; j <= nkl; ++j) {
        MAT(cu, 1, j, 2, *nklmd) = (real)1.0;
        MAT(cu, 2, j, 2, *nklmd) = (real)1.0;
        MAT(iu, 1, j, 2, *nklmd) = 1;
        MAT(iu, 2, j, 2, *nklmd) = 1;
    }
    iphase = 1;
L60:
    if (*m == 0) {
        goto L80;
    }
    for (j = nkl1; j <= nklm; ++j) {
        MAT(cu, 2, j, 2, *nklmd) = (real)1.0;
        MAT(iu, 2, j, 2, *nklmd) = 1;
        jmn = j - *n;
        if (MAT(q, jmn, n2, qrows, qcols) < 0) {
            iphase = 1;
        }
        /* L70: */
    }
L80:
    if (*kode == 0) {
        goto L150;
    }
    for (j = 1; j <= *n; ++j) {
        if (x[j-1] < 0.0) {
            goto L90;
        }
        else if (x[j-1] == 0) {
            goto L110;
        }
        else {
            goto L100;
        }
    L90:
        MAT(cu, 1, j, 2, *nklmd) = (real)1.0;
        MAT(iu, 1, j, 2, *nklmd) = 1;
        goto L110;
    L100:
        MAT(cu, 2, j, 2, *nklmd) = (real)1.0;
        MAT(iu, 2, j, 2, *nklmd) = 1;
    L110:
        ;
    }
    for (j = 1; j <= *k; ++j) {
        jpn = j + *n;
        if (res[j-1] < 0) {
            goto L120;
        }
        else if (res[j-1] == 0) {
            goto L140;
        }
        else {
            goto L130;
        }
    L120:
        MAT(cu, 1, jpn, 2, *nklmd) = (real)1.0;
        MAT(iu, 1, jpn, 2, *nklmd) = 1;
        if (MAT(q, j, n2, qrows, qcols) > 0) {
            iphase = 1;
        }
        goto L140;
    L130:
        MAT(cu, 2, jpn, 2, *nklmd) = (real)1.0;
        MAT(iu, 2, jpn, 2, *nklmd) = 1;
        if (MAT(q, j, n2, qrows, qcols) < 0) {
            iphase = 1;
        }
    L140:
        ;
    }
L150:
    if (iphase == 2) {
        goto L500;
    }
    /* COMPUTE THE MARGINAL COSTS. */
L160:
    for (j = js; j <= n1; ++j) {
        sum = 0;
        for (i = 1; i <= klm; ++i) {
            ii = (int)MAT(q, i, n2, qrows, qcols);
            if (ii < 0) {
                goto L170;
            }
            z__ = MAT(cu, 1, ii, 2, *nklmd);
            goto L180;
        L170:
            iineg = -ii;
            z__ = MAT(cu, 2, iineg, 2, *nklmd);
        L180:
            sum += (double)MAT(q, i, j, qrows, qcols) * (double)z__;
        }
        MAT(q, klm1, j, qrows, qcols) = (real)sum;
    }
    for (j = js; j <= *n; ++j) {
        ii = (int)MAT(q, klm2, j, qrows, qcols);
        if (ii < 0) {
            goto L210;
        }
        z__ = MAT(cu, 1, ii, 2, *nklmd);
        goto L220;
    L210:
        iineg = -ii;
        z__ = MAT(cu, 2, iineg, 2, *nklmd);
    L220:
        MAT(q, klm1, j, qrows, qcols) -= z__;
    }
    /* DETERMINE THE VECTOR TO ENTER THE BASIS. */
L240:
    xmax = 0.0;
    if (js > *n) {
        goto L490;
    }
    for (j = js; j <= *n; ++j) {
        zu = MAT(q, klm1, j, qrows, qcols);
        ii = (int)MAT(q, klm2, j, qrows, qcols);
        if (ii > 0) {
            goto L250;
        }
        ii = -ii;
        zv = zu;
        zu = -zu - MAT(cu, 1, ii, 2, *nklmd) - MAT(cu, 2, ii, 2, *nklmd);
        goto L260;
    L250:
        zv = -zu - MAT(cu, 1, ii, 2, *nklmd) - MAT(cu, 2, ii, 2, *nklmd);
    L260:
        if (kforce == 1 && ii > *n) {
            goto L280;
        }
        if (MAT(iu, 1, ii, 2, *nklmd) == 1) {
            goto L270;
        }
        if (zu <= xmax) {
            goto L270;
        }
        xmax = zu;
        in = j;
    L270:
        if (MAT(iu, 2, ii, 2, *nklmd) == 1) {
            goto L280;
        }
        if (zv <= xmax) {
            goto L280;
        }
        xmax = zv;
        in = j;
    L280:
        ;
    }
    if (xmax <= *toler) {
        goto L490;
    }
    if (MAT(q, klm1, in, qrows, qcols) == xmax) {
        goto L300;
    }
    for (i = 1; i <= klm2; ++i) {
        MAT(q, i, in, qrows, qcols) *= (real)(-1.0);
    }
    MAT(q, klm1, in, qrows, qcols) = xmax;
    /* DETERMINE THE VECTOR TO LEAVE THE BASIS. */
L300:
    if (iphase == 1 || ia == 0) {
        goto L330;
    }
    xmax = 0.0;
    for (i = 1; i <= ia; ++i) {
        z__ = fabs(MAT(q, i, in, qrows, qcols));
        if (z__ <= xmax) {
            goto L310;
        }
        xmax = z__;
        iout = i;
    L310:
        ;
    }
    if (xmax <= *toler) {
        goto L330;
    }
    for (j = 1; j <= n2; ++j) {
        z__ = MAT(q, ia, j, qrows, qcols);
        MAT(q, ia, j, qrows, qcols) = MAT(q, iout, j, qrows, qcols);
        MAT(q, iout, j, qrows, qcols) = z__;
    }
    iout = ia;
    --ia;
    pivot = MAT(q, iout, in, qrows, qcols);
    goto L420;
L330:
    kk = 0;
    for (i = 1; i <= klm; ++i) {
        z__ = MAT(q, i, in, qrows, qcols);
        if (z__ <= *toler) {
            goto L340;
        }
        ++kk;
        res[kk-1] = MAT(q, i, n1, qrows, qcols) / z__;
        s[kk-1] = i;
    L340:
        ;
    }
L350:
    if (kk > 0) {
        goto L360;
    }
    *kode = 2;
    goto L590;
L360:
    xmin = res[0];
    iout = s[0];
    j = 1;
    if (kk == 1) {
        goto L380;
    }
    for (i = 2; i <= kk; ++i) {
        if (res[i-1] >= xmin) {
            goto L370;
        }
        j = i;
        xmin = res[i-1];
        iout = s[i-1];
    L370:
        ;
    }
    res[j-1] = res[kk-1];
    s[j-1] = s[kk-1];
L380:
    --kk;
    pivot = MAT(q, iout, in, qrows, qcols);
    ii = (int)MAT(q, iout, n2, qrows, qcols);
    if (iphase == 1) {
        goto L400;
    }
    if (ii < 0) {
        goto L390;
    }
    if (MAT(iu, 2, ii, 2, *nklmd) == 1) {
        goto L420;
    }
    goto L400;
L390:
    iineg = -ii;
    if (MAT(iu, 1, iineg, 2, *nklmd) == 1) {
        goto L420;
    }
L400:
    ii = abs(ii);
    cuv = MAT(cu, 1, ii, 2, *nklmd) + MAT(cu, 2, ii, 2, *nklmd);
    if (MAT(q, klm1, in, qrows, qcols) - pivot * cuv <= *toler) {
        goto L420;
    }
    /* BYPASS INTERMEDIATE VERTICES. */
    for (j = js; j <= n1; ++j) {
        z__ = MAT(q, iout, j, qrows, qcols);
        MAT(q, klm1, j, qrows, qcols) -= z__ * cuv;
        MAT(q, iout, j, qrows, qcols) = -z__;
        /* L410: */
    }
    MAT(q, iout, n2, qrows, qcols) *= (real)(-1.0);
    goto L350;
    /* GAUSS-JORDAN ELIMINATION. */
L420:
    if (*iter < maxit) {
        goto L430;
    }
    *kode = 3;
    goto L590;
L430:
    ++(*iter);
    for (j = js; j <= n1; ++j) {
        if (j != in) {
            MAT(q, iout, j, qrows, qcols) /= pivot;
        }
    }
    for (j = js; j <= n1; ++j) {
        if (j == in) {
            goto L460;
        }
        z__ = -MAT(q, iout, j, qrows, qcols);
        for (i = 1; i <= klm1; ++i) {
            if (i != iout) {
                MAT(q, i, j, qrows, qcols) += z__ * MAT(q, i, in, qrows, qcols);
            }
        }
    L460:
        ;
    }
    tpivot = -pivot;
    for (i = 1; i <= klm1; ++i) {
        if (i != iout) {
            MAT(q, i, in, qrows, qcols) /= tpivot;
        }
    }
    MAT(q, iout, in, qrows, qcols) = (real)(1.0) / pivot;
    z__ = MAT(q, iout, n2, qrows, qcols);
    MAT(q, iout, n2, qrows, qcols) = MAT(q, klm2, in, qrows, qcols);
    MAT(q, klm2, in, qrows, qcols) = z__;
    ii = (int)fabs(z__);
    if (MAT(iu, 1, ii, 2, *nklmd) == 0 || MAT(iu, 2, ii, 2, *nklmd) == 0) {
        goto L240;
    }
    for (i = 1; i <= klm2; ++i) {
        z__ = MAT(q, i, in, qrows, qcols);
        MAT(q, i, in, qrows, qcols) = MAT(q, i, js, qrows, qcols);
        MAT(q, i, js, qrows, qcols) = z__;
    }
    ++js;
    goto L240;
    /* TEST FOR OPTIMALITY. */
L490:
    if (kforce == 0) {
        goto L580;
    }
    if (iphase == 1 && MAT(q, klm1, n1, qrows, qcols) <= *toler) {
        goto L500;
    }
    kforce = 0;
    goto L240;
    /* SET UP PHASE 2 COSTS. */
L500:
    iphase = 2;
    for (j = 1; j <= nklm; ++j) {
        MAT(cu, 1, j, 2, *nklmd) = 0;
        MAT(cu, 2, j, 2, *nklmd) = 0;
    }
    for (j = n1; j <= nk; ++j) {
        MAT(cu, 1, j, 2, *nklmd) = (real)1.0;
        MAT(cu, 2, j, 2, *nklmd) = (real)1.0;
    }
    for (i = 1; i <= klm; ++i) {
        ii = (int)MAT(q, i, n2, qrows, *nklmd);
        if (ii > 0) {
            goto L530;
        }
        ii = -ii;
        if (MAT(iu, 2, ii, 2, *nklmd) == 0) {
            goto L560;
        }
        MAT(cu, 2, ii, 2, *nklmd) = 0;
        goto L540;
    L530:
        if (MAT(iu, 1, ii, 2, *nklmd) == 0) {
            goto L560;
        }
        MAT(cu, 1, ii, 2, *nklmd) = 0;
    L540:
        ++ia;
        for (j = 1; j <= n2; ++j) {
            z__ = MAT(q, ia, j, qrows, qcols);
            MAT(q, ia, j, qrows, qcols) = MAT(q, i, j, qrows, qcols);
            MAT(q, i, j, qrows, qcols) = z__;
        }
    L560:
        ;
    }
    goto L160;
L570:
    if (MAT(q, klm1, n1, qrows, qcols) <= *toler) {
        goto L500;
    }
    *kode = 1;
    goto L590;
L580:
    if (iphase == 1) {
        goto L570;
    }
    /* PREPARE OUTPUT. */
    *kode = 0;
L590:
    sum = 0.;
    for (j = 1; j <= *n; ++j) {
        x[j-1] = 0;
    }
    for (i = 1; i <= klm; ++i) {
        res[i-1] = 0;
    }
    for (i = 1; i <= klm; ++i) {
        ii = (int)MAT(q, i, n2, qrows, qcols);
        sn = 1.0;
        if (ii > 0) {
            goto L620;
        }
        ii = -ii;
        sn = (real)(-1.0);
    L620:
        if (ii > *n) {
            goto L630;
        }
        x[ii-1] = sn * MAT(q, i, n1, qrows, qcols);
        goto L640;
    L630:
        iimn = ii - *n;
        res[iimn-1] = sn * MAT(q, i, n1, qrows, qcols);
        if (ii >= n1 && ii <= nk) {
            sum += (double)MAT(q, i, n1, qrows, qcols);
        }
    L640:
        ;
    }
    *error = (real)sum;
}

#ifndef MATLAB_MEX_FILE
void cl1_c_float(const int *k, const int *l, const int *m,
                 const int *n, const int *klmd, const int *klm2d,
                 const int *nklmd, const int *n2d, float *q,
                 int *kode, const float *toler, int *iter, float *x,
                 float *res, float *error, float *cu, int *iu, int *s)
{
    cl1<float>(k, l, m, n, klmd, klm2d, nklmd, n2d,
               q, kode, toler, iter, x, res, error, cu, iu, s);
}

void cl1_c_double(const int *k, const int *l, const int *m,
                  const int *n, const int *klmd, const int *klm2d,
                  const int *nklmd, const int *n2d, double *q,
                  int *kode, const double *toler, int *iter, double *x,
                  double *res, double *error, double *cu, int *iu, int *s)
{
    cl1<double>(k, l, m, n, klmd, klm2d, nklmd, n2d,
                q, kode, toler, iter, x, res, error, cu, iu, s);
}
#endif

/******************************************************************************
 * EIGEN MATH LIBRARY INTERFACE (optional)
 ******************************************************************************/

#ifndef MATLAB_MEX_FILE

#ifdef ENABLE_EIGEN_CPP_INTERFACE
template <typename T>
static cl1_result_t cl1_eigen(
    const Matrix<T, Dynamic, Dynamic>& A,
    const Matrix<T, Dynamic, 1>& B,
    Matrix<T, Dynamic, 1>& X,
    const Matrix<T, Dynamic, Dynamic>* C,
    const Matrix<T, Dynamic, 1>* D,
    const Matrix<T, Dynamic, Dynamic>* E,
    const Matrix<T, Dynamic, 1>* F,
    T tolerance,
    int maxiter,
    Matrix<T, Dynamic, 1>* residuals)
{
    if (tolerance <= 0)
    {
        if (sizeof(T) == sizeof(float))
        {
            tolerance = (T)DEFAULT_TOLERANCE_FLOAT;
        }
        else
        {
            tolerance = (T)DEFAULT_TOLERANCE;
        }
    }

    const int k = (int)A.rows();
    const int n = (int)A.cols();
    const int l = (C && D) ? (int)C->rows() : 0;
    const int m = (E && F) ? (int)E->rows() : 0;
    const int klmd = k + l + m;
    const int klm2d = k + l + m + 2;
    const int nklmd = n + k + l + m + 2;
    const int n2d = n + 2;

    if ((k < 1) || (n < 1) || (B.rows() != k))
    {
        return CL1_MATRIX_DIM_INVALID;
    }
    if (C && D)
    {
        if (C->rows() != D->rows() || C->rows() < 1 ||
            C->cols() != n || D->cols() != 1)
        {
            return CL1_MATRIX_DIM_INVALID;
        }
    }
    if (E && F)
    {
        if (E->rows() != F->rows() || E->rows() < 1 ||
            E->cols() != n || F->cols() != 1)
        {
            return CL1_MATRIX_DIM_INVALID;
        }
    }

    if (maxiter <= 0)
    {
        maxiter = 10*(k+l+m); /* default max. iterations */
    }

    Matrix<T, Dynamic, Dynamic> Q(klm2d, n2d);
    Matrix<T, Dynamic, Dynamic> X2d(n2d, 1); /* unknown vector for cl1() */

    Q.setZero();
    Q.block(0,   0, k, n) = A;
    Q.block(0,   n, k, 1) = B;
    if (C && D)
    {
        Q.block(k, 0, l, n) = *C;
        Q.block(k, n, l, 1) = *D;
    }
    if (E && F)
    {
        Q.block(k + l, 0, m, n) = *E;
        Q.block(k + l, n, m, 1) = *F;
    }

    Matrix<T, Dynamic, Dynamic> R(klmd, 1); /* residuals */
    Matrix<T, 2, Dynamic> CU(2, nklmd);
    Matrix<int, 2, Dynamic> IU(2, nklmd);
    Matrix<int, Dynamic, 1> S(klmd, 1);

    X2d.setZero();
    R.setZero();
    CU.setZero();
    IU.setZero();
    S.setZero();

    int kode = 0;
    T error;

    cl1<T>(&k,
           &l,
           &m,
           &n,
           &klmd,
           &klm2d,
           &nklmd,
           &n2d,
           Q.data(),
           &kode,
           &tolerance,
           &maxiter,
           X2d.data(),
           R.data(),
           &error,
           CU.data(),
           IU.data(),
           S.data());

    if (residuals)
    {
        *residuals = R;
    }
    X = X2d.block(0, 0, n, 1);

    return (cl1_result_t)kode;
}

cl1_result_t cl1_float(
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& A,
    const Eigen::Matrix<float, Eigen::Dynamic, 1>& B,
    Eigen::Matrix<float, Eigen::Dynamic, 1>& X,
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>* C,
    const Eigen::Matrix<float, Eigen::Dynamic, 1>* D,
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>* E,
    const Eigen::Matrix<float, Eigen::Dynamic, 1>* F,
    float tolerance,
    int maxiter,
    Eigen::Matrix<float, Eigen::Dynamic, 1>* residuals)
{
    return cl1_eigen<float>(A, B, X, C, D, E, F, tolerance, maxiter, residuals);
}

cl1_result_t cl1_double(
    const Matrix<double, Dynamic, Dynamic>& A,
    const Matrix<double, Dynamic, 1>& B,
    Matrix<double, Dynamic, 1>& X,
    const Matrix<double, Dynamic, Dynamic>* C,
    const Matrix<double, Dynamic, 1>* D,
    const Matrix<double, Dynamic, Dynamic>* E,
    const Matrix<double, Dynamic, 1>* F,
    double tolerance,
    int maxiter,
    Matrix<double, Dynamic, 1>* residuals)
{
    return cl1_eigen<double>(A, B, X, C, D, E, F, tolerance, maxiter, residuals);
}
#endif /* ENABLE_EIGEN_CPP_INTERFACE */

#endif

/******************************************************************************
 * MATLAB MEX INTERFACE (optional)
 ******************************************************************************/

#ifdef MATLAB_MEX_FILE

/* Helper function to copy matrix "in" to matrix "out"
 * at position out_pos_m,n.
 *
 * MATLAB syntax would be:
 * out(out_pos_m:out_pos_m+in_m, out_pos_n:out_pos_n+in_n) = in;
 *
 * in: input matrix
 * in_m: number of rows in "in"
 * in_n: number of columns in "in"
 * out: pointer to output matrix
 * out_m: number of rows in "out"
 * out_n: number of columns in "out"
 * out_pos_m: target location in out
 * out_pos_n: target location in out
 */
static void copymatrix(const double* in, int in_m, int in_n,
                       double* out, int out_m, int out_n,
                       int out_pos_m, int out_pos_n)
{
    if (in_m == 0 || in_n == 0 || in == NULL)
    {
        return; /* ignore empty input matrices */
    }

    /* don't ignore broken input */
    if (out_pos_m + in_m > out_m || out_pos_n + in_n > out_n ||
        in_m > out_m || in_n > out_n || in_m < 0 || in_n < 0)
    {
        mexErrMsgIdAndTxt("JanZwiener:cl1norm:dimension",
                          "Matrix size mismatch");
        return;
    }

    int i, j;
    for (j=0;j<in_n;j++) /* for each column */
    {
        for (i=0;i<in_m;i++) /* for each row */
        {
            MAT(out, i+out_pos_m+1, j+out_pos_n+1, out_m, out_n) =
                MAT(in, i+1, j+1, in_m, in_n);
        }
    }
}

/* The gateway function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const int min_input = 2;
    real toler = (real)DEFAULT_TOLERANCE;
    int kode = 0; /* return value from cl1() function */
    int iter; /* max iterations for cl1.  */
    real err; /* minimum sum of residuals */
    int i;

    // 3 arguments are not valid (if C is given, D is also required).
    // 5 arguments are not valid (if E is given, F is also required).
    if(nrhs < min_input || nrhs == 3 || nrhs == 5) {
        mexErrMsgIdAndTxt(
        "JanZwiener:cl1norm:nrhs",
        "Linear Programming Simplex Solver.\n"
        "Usage: [x, res, info] = cl1norm(A, B, C, D, E, F, tol, maxiter);\n"
        "\n"
        "Input: A, B, C (optional), D (optional),"
        " E (optional), F (optional),\n"
        "tolerance (optional), maxiter (optional).\n"
        "A*x = B, C*x = D, E*x <= F\n"
        "C and D can be empty matrices [] and/or E and F"
        " can be empty matrices [].\n"
        "tolerance: A small positive tolerance. Default: 1e-9.\n"
        "maxiter: Maximum number of iterations for the algorithm.\n"
        "Output: x, residuals (optional), simplexinfo (optional)\n"
        "simplexinfo(1): 0 - optimal solution found. >=1 no solution found.\n"
        "simplexinfo(2): Minimum sum of absolute values of the residuals.\n"
        "simplexinfo(3): Number of simplex iterations.\n"
        "");
    }

    if(nlhs < 1) {
        mexErrMsgIdAndTxt("JanZwiener:cl1norm:nlhs",
                          "One output required.");
    }

    /* make sure the input arguments are real */
    for (i=0;i<nrhs;i++)
    {
        if( !mxIsDouble(prhs[i]) || mxIsComplex(prhs[i]) )
        {
            mexErrMsgIdAndTxt("JanZwiener:cl1norm:notReal",
                              "Input must be real.");
        }
    }

    const mxArray* mA = prhs[0];
    const mxArray* mB = prhs[1];
    const mxArray* mC = (nrhs >= 4) ? prhs[2] : NULL;
    const mxArray* mD = (nrhs >= 4) ? prhs[3] : NULL;
    const mxArray* mE = (nrhs >= 6) ? prhs[4] : NULL;
    const mxArray* mF = (nrhs >= 6) ? prhs[5] : NULL;

    int k = (int)mxGetM(mA);
    int n = (int)mxGetN(mA);
    int l = 0;
    int m = 0;

    if (nrhs >= 4)
    {
        l = (int)mxGetM(mC);
    }
    if (nrhs >= 6)
    {
        m = (int)mxGetM(mE);
    }
    if (nrhs >= 7)
    {
        toler = mxGetPr(prhs[6])[0];
    }
    if (nrhs >= 8)
    {
        iter = (int)mxGetPr(prhs[7])[0];
    }
    else
    {
        iter = 10*(k+l+m); /* default max. iterations */
    }

    const int klmd = k + l + m;
    const int klm2d = k + l + m + 2;
    const int nklmd = n + k + l + m + 2;
    const int n2d = n + 2;

    if (k < 1 || n < 1)
    {
        mexErrMsgIdAndTxt("JanZwiener:cl1norm:dimension",
                          "Matrix A empty.");
    }

    if ((int)mxGetM(mB) != k ||
        (int)mxGetN(mB) != 1)
    {
        mexErrMsgIdAndTxt("JanZwiener:cl1norm:dimension",
                          "B matrix dimension does not match A matrix.");
    }

    if (l > 0)
    {
        if ((int)mxGetN(mC) != n ||
            (int)mxGetM(mD) != l ||
            (int)mxGetN(mD) != 1)
        {
            mexErrMsgIdAndTxt("JanZwiener:cl1norm:dimension",
                              "Input dimension mismatch.");
        }
    }

    if (m > 0)
    {
        if ((int)mxGetN(mE) != n ||
            (int)mxGetM(mF) != m)
        {
            mexErrMsgIdAndTxt("JanZwiener:cl1norm:dimension",
                              "Input dimension mismatch.");
        }
    }

    // prepare input data
    mxArray* mQ      = mxCreateDoubleMatrix(klm2d, n2d,   mxREAL);
    mxArray* mX      = mxCreateDoubleMatrix(n2d,   1,     mxREAL);
    mxArray* mXsmall = mxCreateDoubleMatrix(n,     1,     mxREAL);
    mxArray* mRES    = mxCreateDoubleMatrix(klmd,  1,     mxREAL);
    mxArray* mCU     = mxCreateDoubleMatrix(2,     nklmd, mxREAL);
    int* AUI         = (int*)mxMalloc(2*nklmd*sizeof(int));
    int* outS        = (int*)mxMalloc(klmd*sizeof(int));
    double* AQ       = mxGetPr(mQ);
    double* AX       = mxGetPr(mX);
    double* AXsmall  = mxGetPr(mXsmall);
    double* ARES     = mxGetPr(mRES);
    double* ACU      = mxGetPr(mCU);
    const double* AA = mxGetPr(mA);
    const double* AB = mxGetPr(mB);
    const double* AC = mC ? mxGetPr(mC) : NULL;
    const double* AD = mD ? mxGetPr(mD) : NULL;
    const double* AE = mE ? mxGetPr(mE) : NULL;
    const double* AF = mF ? mxGetPr(mF) : NULL;

    // copy all the input matrices in the big Q matrix.
    //     +------+
    // Q = | A B  |
    //     | C D  |
    //     | E F  |
    //     |      |
    //     +------+
    copymatrix(AA, k, n, AQ, klm2d, n2d, 0,   0);
    copymatrix(AB, k, 1, AQ, klm2d, n2d, 0,   n);
    copymatrix(AC, l, n, AQ, klm2d, n2d, k,   0);
    copymatrix(AD, l, 1, AQ, klm2d, n2d, k,   n);
    copymatrix(AE, m, n, AQ, klm2d, n2d, k+l, 0);
    copymatrix(AF, m, 1, AQ, klm2d, n2d, k+l, n);

    // actual calculation
    cl1(&k, &l, &m, &n, &klmd, &klm2d, &nklmd, &n2d,
        AQ, &kode, &toler, &iter, AX, ARES, &err, ACU, AUI, outS);

    /* X has additional fields for the cl1 function. we only need
     * the first n-values */
    copymatrix(AX, n, 1, AXsmall, n, 1, 0, 0);
    plhs[0] = mXsmall;

    /* return residuals? */
    if (nlhs > 1)
    {
        plhs[1] = mRES;
    }
    else
    {
        mxDestroyArray(mRES);
    }

    /* return dbg info? */
    if (nlhs > 2)
    {
        mxArray* mSIMPLEXINFO = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* ASIMPLEXINFO  = mxGetPr(mSIMPLEXINFO);
        ASIMPLEXINFO[0] = (double)kode;
        ASIMPLEXINFO[1] = (double)err;
        ASIMPLEXINFO[2] = (double)iter;

        plhs[2] = mSIMPLEXINFO;
    }

    // cleanup
    mxDestroyArray(mX);
    mxDestroyArray(mQ);
    mxDestroyArray(mCU);
    mxFree(AUI);
    mxFree(outS);
}

#endif /* MATLAB_MEX_FILE */

/* @} */
