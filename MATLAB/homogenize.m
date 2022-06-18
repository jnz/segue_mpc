function [ldash, Adash, Lw] = homogenize(l, Qll, A)
% Compute a modified measurement vector l and a modified
% design matrix A, so that the covariance matrix Qll is not needed.
%
% For a least-squares problem:
% l = A*x
%
% ldash = Lw*l;
% Adash = Lw*A;
%
% with Qll as the covariance of l
% Qll = L*L' (Cholesky decomposition)
%
% P = inv(Qll)
%
% inv(Qll) = (L*L')^(-1) (i.e. inv(L*L'))
% inv(Qll) = (L')^(-1)*L^(-1)
% inv(Qll) = (L^(-1))'*L^(-1) = P
if nargin ~= 3
    error('Usage: [ldash, Adash, Lw] = homogenize(l, Qll, A).');
end

%% error checks
if ~is_covariance_valid(Qll)
    error('Covariance matrix Qll is invalid.');
end

%% cholesky decomposition
L = chol(Qll, 'lower'); % L*L' = Qll

% mldivide checks for triangular matrices
n = size(Qll,1);
Lw = L\eye(n); % Lw = inv(L);

ldash = Lw*l;
Adash = Lw*A;

end

function [valid] = is_covariance_valid(Q)

%% check main diagonal line (has to be positive)
min_element = min(diag(Q));
if (min_element < eps)
    valid = false;
    return;
end
%% covariance has to be symmetric
if (~issymmetric(Q))
    valid = false;
    return;
end

%% all checks passed
valid = true;
end

