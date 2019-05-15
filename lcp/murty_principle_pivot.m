function [A, b, x, w, return_code] = murty_pivot(A, b, x, w, s)

% TODO: don't need s input arg anymore
% TODO: implement a starting S, instead of S_init = zeros(n, 1)

% Sanity check input
% A = n x n matrix, assume SPD
[n m] = size(A);
assert(n == m)
% b = n x 1 vector, includes all elements in S and S-complement
m = size(b);
assert(n == m(1) && m(2) == 1)
% w = n x 1 vector, elements 1 to (s-1) are zeros, elements s to n are positive
% Cannot have w without s
assert(nargin == 2 || nargin == 5);
if nargin == 5
  % Check x
  m = size(x);
  assert(n == m(1) && m(2) == 1);
  for ii = 1:s-1
    assert(x(ii) > 0)
  end
  assert(sum(x(s:n)) == 0)
  % Check w
  m = size(w);
  assert(n == m(1) && m(2) == 1);
  assert(sum(w(1:s-1)) == 0)
  for ii = s:n
    assert(w(ii) > 0)
  end
end

% Starting with S = empty
S = logical(zeros(n, 1));
x = zeros(n, 1);
w = -b;

% TODO: how to guard against infinite loops
max_iterations = min(1000, 2^n);
iteration_count = 0;
while iteration_count < max_iterations
  [check S] = check_solution(A, b, x, w, S);
  if ~check
    x(S) = A(S,S) \ b(S);
    x(~S) = zeros(sum(~S), 1);
    w(~S) = A(~S,S) * x(S) - b(~S);
    w(S) = zeros(sum(S), 1);
  else
    break;
  end
  iteration_count = iteration_count + 1;
end

% Function exit checks and error code
if iteration_count >= max_iterations
  disp('ERROR: iteration_count exceeded max_iterations %d, exit.', max_iterations);
  return_code = 1;
elseif (~check_solution(A, b, x, w, S))
  disp('ERROR: check_solution returns false, check algorithm.');
  return_code = 2;
else
  return_code = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% If S does not satisfy LCP conditions for x and w, return an updated S,
% else, check the equation Ax = b+w holds.

function [check S] = check_solution(A, b, x, w, S)

% TODO: x(S) and w(~S) <= 0 or just < 0?
if sum(x(S) <= 0) > 0
  check = false;
  offending_elem_in_s = find(S, find(x(S) <= 0, 1));
  S(offending_elem_in_s(end)) = false;
elseif sum(w(~S) <= 0) > 0
  check = false;
  offending_elem_in_s_complement = find(~S, find(w(~S) <= 0, 1));
  S(offending_elem_in_s_complement(end)) = true;
else
  numerical_tolerance = 1e-9;
  lhs = A*x;
  rhs = b+w;
  if sum(abs(lhs-rhs)) > numerical_tolerance
    check = false;
    disp('ERROR: Found solution does not satisfy equation Ax = b+w');
  end
  check = true;
end

% TODO: check the check_solution with intentionally wrong things.