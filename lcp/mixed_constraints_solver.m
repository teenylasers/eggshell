function [x, w, return_code] = mixed_constraints_solver(A, b, C)

% A = n x n matrix, assume SPD
[n m] = size(A);
assert(n == m)

A_ee = A(C, C);
A_ei = A(C, ~C);
A_ie = A(~C, C);
A_ii = A(~C, ~C);
b_e = b(C);
b_i = b(~C);

% solve for x_i using lcp
lcp_lhs = A_ii - A_ie * inv(A_ee) * A_ei;
lcp_rhs = b_i - A_ie * inv(A_ee) * b_e;
[A_, b_, x_i, w_i, ret] = murty_principal_pivot (lcp_lhs, lcp_rhs);
if ret ~= 0
  return_code = ret;
end

% solve for x_e
x_e = inv(A_ee) * (b_e - A_ei * x_i);

% construct x, w
x = zeros(n, 1);
w = zeros(n, 1);
x(C) = x_e;
x(~C) = x_i;
w(~C) = w_i;

% check solution
numerical_tolerance = 1e-9;
lhs = A * x;
rhs = b + w;
if sum(abs(lhs-rhs)) > numerical_tolerance
  disp('ERROR: Found solution does not satisfy equation Ax = b+w');
  return_code = 3;
else
  fprintf('sum(abs(lhs - rhs)) = %d\n', sum(abs(lhs-rhs)));
  return_code = 0;
end