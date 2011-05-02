function d = distptln(x0,y0,x1,y1,x2,y2)
%% distptln 
% distance from a point (x0,y0) to the line connecting (x1,y1) and (x2,y2)
%% Inputs
% * (x0,y0) : the position of the point 'p'
% * (x1,y1) : the origin of the line L1
% * (x2,y2) : the tip of the line L1
%% Outputs
% * d    : distance from p to L1
%%   Copyright 
%   Karl Ji-Wung Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.1.1.1 $  $Date: 2009/07/02 $
%
    d = abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/sqrt((x2-x1)^2+(y2-y1)^2);
end