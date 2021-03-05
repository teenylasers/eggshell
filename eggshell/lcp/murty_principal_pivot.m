function [x, w, return_code] = murty_principal_pivot(A, b, x_lo, x_hi)

assert(nargin == 2 || nargin == 4);

% Sanity check input
% A = n x n matrix, assume SPD
[n m] = size(A);
assert(n == m)
% b = n x 1 vector
m = size(b);
assert(n == m(1) && m(2) == 1)

% If x_lo and x_hi are not specified, then standard LCP problem, x >= 0
if nargin == 2
  x_lo = 0;
  x_hi = Inf;
end

% Check that
% 1. x_lo < x_hi
% 2. x_lo <= 0 and x_hi > 0
% TODO: why does the algorithm not work for x_lo > 0? and should that be the case?
assert(x_lo < x_hi)
assert(x_lo <= 0 && x_hi > 0)

% S indicates whether the solution element lies on
%  S : x_lo <= x <= x_hi, w = 0, or
% ~S : x = x_hi, w <= 0, or x = x_lo, w >= 0
% Starting with S = 1 for all elements of S
S = ones(n, 1) == 1;
x = zeros(n, 1);
w = -b;
% For elements in ~S, C indicates which domain w(i) should be, whether it is C(i) == x_lo
% && w >= 0, or C(i) == x_hi && w <= 0.
C = ones(n, 1) * x_lo;

% TODO: how to guard against infinite loops
max_iterations = min(1000, 2^n);
iteration_count = 0;
while iteration_count < max_iterations
  [check S C] = check_solution(A, b, x, w, S, C, x_lo, x_hi);
  if ~check
    x(S) = A(S,S) \ b(S);
    x(~S & (C == x_lo)) = ones(sum(~S & (C == x_lo)), 1) * x_lo;
    x(~S & (C == x_hi)) = ones(sum(~S & (C == x_hi)), 1) * x_hi;
    w(~S) = A(~S,S) * x(S) - b(~S);
    w(S) = zeros(sum(S), 1);
  else
    break;
  end
  iteration_count = iteration_count + 1;
end

% Function exit checks and error code
if iteration_count >= max_iterations && ~check_solution(A,b,x,w,S,C,x_lo,x_hi)
  fprintf(['ERROR: iteration_count exceeded max_iterations %d without ', ...
           'finding a valid solution, exit.\n'], max_iterations);
  return_code = 1;
elseif (~check_solution(A, b, x, w, S, C, x_lo, x_hi))
  disp('ERROR: check_solution returns false, check algorithm.');
  return_code = 2;
else
  return_code = 0;
end

if return_code ~= 0
  fprintf('x_lo = %d, x_hi = %d\n', x_lo, x_hi);
  %x
  %w
  %S
  %C
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% If S does not satisfy LCP conditions for x and w, return an updated S,
% else, check the equation Ax = b+w holds.

function [check S C] = check_solution(A, b, x, w, S, C, x_lo, x_hi)

dim = length(S);
check = false;

for i = 1:dim
  % Check x(S),
  % 1. if x(i) < x_lo, then S(i) = false, C(i) = x_lo
  % 2. if x(i) > x_hi, then S(i) = false, C(i) = x_hi
  if S(i)
    if x(i) < x_lo
      S(i) = false;
      C(i) = x_lo;
      return;
    elseif x(i) > x_hi
      S(i) = false;
      C(i) = x_hi;
      return;
    end
  % Check w(~S),
  % 1. if C(i) = x_lo && w(i) < 0, then S(i) = true
  % 2. if C(i) = x_hi && w(i) > 0, then S(i) = true
  else
    if C(i) == x_lo && w(i) < 0
      S(i) = true;
      return;
    elseif C(i) == x_hi && w(i) > 0
      S(i) = true;
      return;
    end
  end
end

% Passed both x(S) and w(~S) checks, check goodness of solution
numerical_tolerance = 1e-9;
if any(x < x_lo) || any(x > x_hi)
  %disp('LOG: found x elements that do not satisfy x_lo <= x <= x_hi');
  return;
end
if any(w(x == x_lo) < 0) || any(w(x == x_hi) > 0)
  %disp('LOG: found w elements that do not satisfy w > 0 for x = x_lo || w < 0 for x = x_hi');
  return;
end
lhs = A*x;
rhs = b+w;
if sum(abs(lhs-rhs)) > numerical_tolerance
  disp('ERROR: Found solution does not satisfy equation Ax = b+w');
else
  check = true;
end

% TODO: check the check_solution with intentionally wrong things.
