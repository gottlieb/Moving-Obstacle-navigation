function [x,y] = sample_points( p, vmin, vmax, thl, thr )

%% sample_points
% Quantize the allowable velocity vectors in discrete system, given current
% velocity of the robot which is located at 'p'. The allowable velocity range
% is circular sector constrained by the minimum and maximum velocity 'vmin'
% and 'vmax', and the left and right heading 'thl' and 'thr'.
%% Syntax
% * sample_points( p, vmin, vmax, thl, thr )
%% Inputs
% * p    : the location of the robot
% * vmin : the minimum allowable velocity of the robot
% * vmax : the maximum allowable velocity of the robot
% * thl  : the leftmost allowable heading of the robot
% * thr  : the rightmost allowable heading of the robot
%% Outputs
% * (x,y) : the array of the sampled points
%%   Copyright 
%   Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.5.1.1$  $Date: 2010/11/16$
%

    n = 5;
    m = 3;
    x = [];
    y = [];
    if thl < thr
        thl = thl + 2*pi;
    end
    theta = linspace( thr, thl, n );
    for i=1:n
        p0 = p + vmin*[cos(theta(i)),sin(theta(i))];
        p1 = p + vmax*[cos(theta(i)),sin(theta(i))];
        x = [x, linspace( p0(1), p1(1), m )];
        y = [y, linspace( p0(2), p1(2), m )];
    end
end