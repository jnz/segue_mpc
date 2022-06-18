#ifndef __MPCGAIN_H
#define __MPCGAIN_H

#include "Eigen/Dense"
/*
% Compute MPC gain matrices
%
% Basic model (inputs):
%
%   x(k+1) = Ap*x(k) + Bp*u(k)
%   y(k)   = Cp*x(k)
%
% Np = Prediction epochs
% Nc = Control Epochs (<= Np)
% x(k) = System state at epoch k
% u(k) = Manipulated variable / input variable at epoch k
% y(k) = Process output at epoch k
%
% Outputs:
% Phi Np x Nc matrix (influence of control input on future states)
% F Np x length(x) matrix (state prediction)
%
% Prediction of state(s) y over Np epochs:
%
% Y = F*x(k) + Phi*U
%
% with:
% Y = [ y(k+1); y(k+2); y(k+3); ... ; y(k+Np) ]
% U = [ u(k);   u(k+1); u(k+2); ... ; u(k + Nc - 1) ]
%
% F = [   Cp*Ap ;
%         Cp*Ap*Ap ;
%         Cp*Ap*Ap*Ap ;
%           ...
%         Cp*Ap^Np ];
%
% Phi = [ Cp*Bp            0               0               ... 0 ;
%         Cp*Ap*Bp         Cp*Bp           0               ... 0 ;
%         Cp*Ap^2*Bp       Cp*Ap*Bp        Cp*Bp           ... 0 ;
%          ...
%         Cp*Ap^(Np-1)*Bp  Cp*Ap^(Np-2)*Bp Cp*Ap^(Np-3)*Bp ... Cp*Ap^(Np-Nc)*Bp ];
%
% Reference / desired / setpoint state(s) y for the next Np epochs:
%
% Rs = [ setpoint_y(k+1); setpoint_y(k+2); ... ; setpoint_y(k+Np) ]
%
% If only a single setpoint is desired for the next Np epochs:
% Rs = ones(Np,1)*setpoint_y
%
% Solve for U:
%
% Rs - F*x(k) = Phi*U
%
% If a cost for control effort is to be added, the following
% cost function minimizes errors between predicted state and desired
% state (first term) and minimize control effort (keep U small):
% J = (Rs − Y)'*(Rs − Y) + U'*Rbar*U
%
% with:
%
%  Rbar = rw*eye(Nc); % rw = scalar tuning parameter, 0 means no attention
%                            to control effort. Higher values mean
%                            a higher penalty on control effort.
%
% Extending with Y = F*x(k) + Phi*U:
%
% J = (Rs - F*x)'*(Rsbar - F*x) - 2*U'*Phi'*(Rs - F*x)) +
%                                   U'*(Phi'*Phi + Rbar)*U
%
% Least squares solution without constraints:
%
% U = (Phi'*Phi + Rbar)\(Phi'*(Rs - F*x))
%
% with quadprog(...):
%
% U = quadprog((Phi'*Phi + Rbar), -Phi'*(Rs - F*x), [], [])
%
% New control input (use first DU, discard the rest of the DU vector):
% u = U(1)
*/

void mpcgain(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Ap,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Bp,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Cp,
    const unsigned int Nc,
    const unsigned int Np,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Phi,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& F);

/** @brief Compute linear MPC gain matrices for an extended state model
*
* Basic model (inputs):
*
*   x(k+1) = Ap*x(k) + Bp*u(k)
*   y(k)   = Cp*x(k)
*
* Np = Prediction epochs
* Nc = Control Epochs (<= Np)
* x(k) = System state at epoch k
* u(k) = Manipulated variable / input variable at epoch k
* y(k) = Process output at epoch k
*
* Outputs:
* Phi Np x Nc matrix (influence of control input on future states)
* F Np x length(x) matrix (state prediction)
*
* An extended/augmented state model is used:
*
*   dx(k) = x(k) - x(k-1)
*   du(k) = u(k) - u(k-1)
*   dx(k+1) = Ap*dx(k) + Bp*du(k)
*   y(k+1)  = Cp*Ap*dx(k) + Cp*Bp*du(k) + y(k) 
*
*   x_e(k) = [ dx(k); y(k) ]
*
* Extended/augmented model (denoted by underscore _e):
*
*    x_e(k+1) = A_e*x_e(k) + B_e*du(k)
*    A_e = [ Ap     0; 
*            Cp*Ap  1 ];
*    B_e = [ Bp ;
*            Cp*Bp ];
*    
*    y(k) = C_e*x_e(k) = [ 0 Cp ]*x_e(k)
*
* Prediction of state(s) y over Np epochs:
*
* Y = F*x(k) + Phi*DU
*
* with:
* Y = [ y(k+1); y(k+2); y(k+3); ... ; y(k+Np) ]
* DU = [ du(k); du(k+1); du(k+2); ... ; du(k + Nc - 1) ]
*
* F = [   C_e*A_e ;
*         C_e*A_e*A_e ;
*         C_e*A_e*A_e*A_e ;
*           ...
*         C_e*A_e^Np ];
*
* Phi = [ C_e*B_e             0                  0                  ... 0 ;
*         C_e*A_e*B_e         C_e*B_e            0                  ... 0 ;
*         C_e*A_e^2*B_e       C_e*A_e*B_e        C_e*B_e            ... 0 ;
*          ...
*         C_e*A_e^(Np-1)*B_e  C_e*A_e^(Np-2)*B_e C_e*A_e^(Np-3)*B_e ... C_e*A_e^(Np-Nc)*B_e ];
*
* Desired / setpoint state(s) y for the next Np epochs:
*
* Rs = [ setpoint_y(k+1); setpoint_y(k+2); ... ; setpoint_y(k+Np) ]
*
* If only a single setpoint is desired for the next Np epochs:
* Rs = ones(Np,1)*setpoint_y
*
* Solve for DU:
*
* Rs - F*x(k) = Phi*DU
*
* If a cost for control effort is to be added, the following
* cost function minimizes errors between predicted state and desired
* state (first term) and minimize control effort (keep DU small): 
* J = (Rs − Y)'*(Rs − Y) + DU'*Rbar*DU
*
* with:
*
*  Rbar = rw*eye(Nc); % rw = scalar tuning parameter, 0 means no attention
*                            to control effort. Higher values mean
*                            a higher penalty on control effort.
*
* Extending with Y = F*x(k) + Phi*DU:
*
* J = (Rs - F*x_e)'*(Rsbar - F*x_e) - 2*DU'*Phi'*(Rs - F*x_e)) +
*                                     DU'*(Phi'*Phi + Rbar)*DU
*
* Least squares solution without constraints:
*
* DU = (Phi'*Phi + Rbar)\(Phi'*(Rs - F*x_e))
*
* with quadprog(...):
*
* DU = quadprog(2*(Phi'*Phi + Rbar), -2*Phi'*(Rs - F*x_e), [], [])
*
* New control input (use first DU, discard the rest of the DU vector):
* u = u + DU(1)
*
*/

void mpcgainEx(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Ap,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Bp,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Cp,
    const unsigned int Nc,
    const unsigned int Np,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& Phi,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& F);

#endif
