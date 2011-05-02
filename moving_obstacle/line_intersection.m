function [x,y,tf] = line_intersection(x1,y1,x2,y2,x3,y3,x4,y4)

%% line_intersection
% computes (x,y), the intersection of two lines L1 and L2 in two dimensions with,
% L1 containing the points (x1,y1) and (x2,y2), and L2 containing the points (x3,y3)
% and (x4,y4).
%% Inputs
% * (x1,y1) : the origin of the line L1
% * (x2,y2) : the tip of the line L1
% * (x3,y3) : the origin of the line L2
% * (x4,y4) : the tip of the line L2
%% Outputs
% * (x,y)   : the intersection of two lines L1 and L2
%% References
% [1] <http://mathworld.wolfram.com/Line-LineIntersection.html Line-Line
% Intersection -- from Wolfram MathWorld>

    x = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))...
        /((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)); 
    y = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))...
        /((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)); 

    v1 = [x-x1,y-y1];
    v2 = [x-x2,y-y2];
    dv1 = dot(v1,v2);
    v1 = [x-x3,y-y3];
    v2 = [x-x4,y-y4];
    dv2 = dot(v1,v2);
    if dv1 <= 0 && dv2 <= 0  % intersection is on the edge
        tf = 1;
    else 
        tf = 0;
    end
end