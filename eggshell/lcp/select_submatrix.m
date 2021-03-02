% Test the manual implementation of A(S,S), where A is a square matrix, S is a boolean vector

function sm = select_submatrix(A, S)

n = size(A,1);     % Dimension of input argument

% not correct
% sd = sum(S);       % submatrix dimensions
% sm = zeros(sd,sd); % initialize submatrix
% si = 1;            % advancing submatrix index
% sb = 0;            % current subblock size
% for i = 1:n
%   if S(i)
%     sb = sb + 1;
%   end
%   if (i == n) || (~S(i+1) && sb)
%     i
%     si
%     sb = sb - 1
%     sm(si:si+sb, si:si+sb) = A(i-sb:i, i-sb:i)
%     sm(si:si+sb, 1) = A(i-sb:i, 1)
%     sm(1, si:si+sb) = A(1, i-sb:i)
%     si = si + 1;
%     sb = 0;
%   end
% end

sm_i = 1;
sm_j = 1;
for i = 1:size(A,1)
  for j = 1:size(A,1)
    if S(j)
      if S(i)
	sm(sm_i, sm_j) = A(i, j);
      end
      sm_j = sm_j + 1;
    end
  end
  sm_j = 1;
  if S(i)
    sm_i = sm_i + 1;
  end
end
