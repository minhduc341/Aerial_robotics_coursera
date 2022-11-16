function [T] = polyT(n, k, t)
% One utility function we are going to build to help us with the above is 
% creating the polynom coefficient-coefficient vector (for lack of better name,
% these are the actual values you would put into the matrix raws). 
% To understand what this mean here is an example: Lets say I want to get a vector of 8 variables 
% (for a 7th order polynom) for the first derivative when t=1. 
% This utility function should return a vector of: 0 1 2 3 4 5 6 7. 
% When we build matrix A we will use this utility function to create those vector for us.
% n is the polynom number of coefficients, k is the requested derivative and t is the actual value of t 
% (this can be anything, not just 0 or 1).
T = zeros(n,1);
D = zeros(n,1);

% Init:
for i=1:n
    D(i) = i-1;
    T(i) = 1;
end

% Derivative:
for j=1:k
    for i=1:n
        T(i) = T(i) * D(i);
        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end

% put t value
for i=1:n
    T(i) = T(i) * t^D(i);
end

T = T';

end