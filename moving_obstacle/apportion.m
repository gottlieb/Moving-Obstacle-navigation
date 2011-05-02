% dist = apportion(m)
%
% This is a support function for the Monte Carlo simulation of Choi's
% velocity obstacle demonstration. Given a number of points (m),
% apportion() generates every possible apportionment of those points across
% the three types of obstacles - static, linear movement, and circular
% movement.
%
% What is returned is simply a matrix with 3 columns (one for each obstacle
% type) and (m+1)(m+2)/2 rows, one for each distribution.
%
% Written by Jeremy Gottlieb, Autonomous Systems Lab, UCSC
% Last modified: 4/22/2011

function dist = apportion(m)
    row = 1;
    for i=0:m
        for j=0:m-i
            dist(row,1) = i;
            dist(row,2) = j;
            dist(row,3) = m-(i+j);
            row = row + 1;
        end
    end
end