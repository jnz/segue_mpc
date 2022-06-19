#ifndef __CL1NORM_H
#define __CL1NORM_H

/*
 * Interface to the CL1NORM module.
 * ================================
 *
 */

/* THIS FUNCTION USES A MODIFICATION OF THE SIMPLEX
   METHOD OF LINEAR PROGRAMMING TO CALCULATE AN L1 SOLUTION
   TO A K BY N SYSTEM OF LINEAR EQUATIONS
               AX=B
   SUBJECT TO L LINEAR EQUALITY CONSTRAINTS
               CX=D
   AND M LINEAR INEQUALITY CONSTRAINTS
               EX.LE.F. (E*X <= F)
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

#ifdef __cplusplus
extern "C" {
#endif

/* Be aware that the float (32-bit) version ideally needs a well
   conditioned A matrix. If in doubt, use the double version. */
void cl1_c_float(const int *k, const int *l, const int *m,
                 const int *n, const int *klmd, const int *klm2d,
                 const int *nklmd, const int *n2d, float *q,
                 int *kode, const float *toler, int *iter, float *x,
                 float *res, float *error, float *cu, int *iu, int *s);

void cl1_c_double(const int *k, const int *l, const int *m,
                  const int *n, const int *klmd, const int *klm2d,
                  const int *nklmd, const int *n2d, double *q,
                  int *kode, const double *toler, int *iter, double *x,
                  double *res, double *error, double *cu, int *iu, int *s);

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus

#include "Eigen/Dense"

typedef enum cl1_result_enum
{
    CL1_OPT_SOL_FOUND      = 0, /* optimal solution found */
    CL1_NO_SOL_FOUND       = 1, /* no feasible solution to the constraints, */
    CL1_ROUNDING_ERRORS    = 2, /* calculations terminated prematurely due to rounding errors */
    CL1_MAX_ITER           = 3, /* maximum number of iterations reached */
    CL1_MATRIX_DIM_INVALID = 4, /* maximum number of iterations reached */

    CL1_RESULT_MAX
} cl1_result_t;

cl1_result_t cl1_float(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& A,
                       const Eigen::Matrix<float, Eigen::Dynamic, 1>& B,
                       Eigen::Matrix<float, Eigen::Dynamic, 1>& X,
                       const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>* C = NULL,
                       const Eigen::Matrix<float, Eigen::Dynamic, 1>* D = NULL,
                       const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>* E = NULL,
                       const Eigen::Matrix<float, Eigen::Dynamic, 1>* F = NULL,
                       float tolerance = 0,
                       int maxiter = 0,
                       Eigen::Matrix<float, Eigen::Dynamic, 1>* residuals = NULL);

cl1_result_t cl1_double(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& A,
                        const Eigen::Matrix<double, Eigen::Dynamic, 1>& B,
                        Eigen::Matrix<double, Eigen::Dynamic, 1>& X,
                        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>* C = NULL,
                        const Eigen::Matrix<double, Eigen::Dynamic, 1>* D = NULL,
                        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>* E = NULL,
                        const Eigen::Matrix<double, Eigen::Dynamic, 1>* F = NULL,
                        double tolerance = 0,
                        int maxiter = 0,
                        Eigen::Matrix<double, Eigen::Dynamic, 1>* residuals = NULL);


#endif /* __cplusplus */

#endif /* __CL1NORM_H */
