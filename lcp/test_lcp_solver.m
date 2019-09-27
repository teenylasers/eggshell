function test_lcp_solver(n, num_tests)
% n = size of A and b.

num_tests_failed = 0;

% Murty pivot solver with x >= 0
fprintf('=== Murty Principal Pivot, x >= 0 ===\n');
for ii = 1:num_tests
  ret = test_murty_pivot(n);
  if ret ~= 0
    num_tests_failed = num_tests_failed + 1;
  end
end
fprintf('%d out of %d tests passed.\n', num_tests - num_tests_failed, num_tests);

% Murty pivot solver with {x_lo, x_hi} limits
fprintf('=== Murty Principal Pivot, {x_lo, x_hi} limits ===\n');
for ii = 1:num_tests
  ret = test_murty_pivot_xlimits(n);
  if ret ~= 0
    num_tests_failed = num_tests_failed + 1;
  end
end
fprintf('%d out of %d tests passed.\n', num_tests - num_tests_failed, num_tests);

% Mixed constraint solver
fprintf('=== Mixed constraint solver, x >= 0 ===\n');
for ii = 1:num_tests
  ret = test_mixed_constraints_solver(n);
  if ret ~= 0
    num_tests_failed = num_tests_failed + 1;
  end
end
fprintf('%d out of %d tests passed.\n', num_tests - num_tests_failed, num_tests);

% TODO: test mixed constraint solver with different {x_lo, x_hi}

function ret = test_murty_pivot(n)
A = generate_spd_matrix(n);
b = rand(n,1);
[x, w, ret] = murty_principal_pivot(A, b);

function ret = test_murty_pivot_xlimits(n)
A = generate_spd_matrix(n);
b = rand(n,1);
x_limits = rand(1,2);
x_lo = x_limits(1) * -1;
x_hi = x_limits(2);
[x, w, ret] = murty_principal_pivot(A, b, x_lo, x_hi);

function ret = test_mixed_constraints_solver(n)
A = generate_spd_matrix(n);
b = rand(n,1);
C = rand(n,1) < 0.5;
[x, w, ret] = mixed_constraints_solver(A, b, C);

function A = generate_spd_matrix(n)
% Generate a random SPD matrix A of size (n x n)
% TODO: rand generates elements between [0,1], test negative numbers?
% TODO: think through again, why m'*m gives a SPD matrix
m = rand(n);
A = m'*m;
