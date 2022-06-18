function [Phi, F] = linearmpcgains(Ap,Bp,Cp,Nc,Np)
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

assert(Np >= Nc);
assert(Nc > 0);
warning('This code is still experimental');

[m1,~]=size(Cp); % number of setpoint state variables
[n1,n_in]=size(Bp); % n1: state vector size, n_in number of control inputs
assert(n1 >= m1);

% Build F matrix:
% F = [   Cp*Ap ;
%         Cp*Ap*Ap ;
%         Cp*Ap*Ap*Ap ;
%           ...
%         Cp*Ap^Np ];

F = zeros(Np*m1, n1);
F(1:m1,:)=Cp*Ap;

h = zeros(Np*m1, n1); % helper matrix to build Phi later
h(1:m1,:)=Cp;
jj = m1+1;
for kk=2:Np
    F(jj:jj+m1-1,:)=F(jj-m1:jj-1,:)*Ap;
    h(jj:jj+m1-1,:)=h(jj-m1:jj-1,:)*Ap;
    jj = jj + m1;
end

%
% Phi = [ Cp*Bp            0               0               ... 0 ;
%         Cp*Ap*Bp         Cp*Bp           0               ... 0 ;
%         Cp*Ap^2*Bp       Cp*Ap*Bp        Cp*Bp           ... 0 ;
%          ...
%         Cp*Ap^(Np-1)*Bp  Cp*Ap^(Np-2)*Bp Cp*Ap^(Np-3)*Bp ... Cp*Ap^(Np-Nc)*Bp ];
%
v=h*Bp;
Phi=zeros(Np*m1,Nc);
Phi(:,1)=v;
% first column of Phi
stride = m1;
for i=2:Nc
    Phi(:,i)=[zeros(stride,1);v(1:Np*m1-stride,1)]; % Toeplitz matrix
    stride = stride + m1;
end

end
