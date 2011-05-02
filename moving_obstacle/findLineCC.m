function [thcl,thcr] = findLineCC( p, po )

%% findLineCC
% Find the two lines to determine collision cone of the robot with the
% location 'p' relative to the obstacle with the location 'po.'
%% Inputs
% * p   : the location of the robot
% * po  : the location of the obstacle
%% Outputs
% * thcl : angle of the left line enclosing the collision cone
% * thcr : angle of the right line enclosing the collision cone
%%   Copyright 
%   Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.5.1.1$  $Date: 2010/11/16$
%

    th1 = atan2( po(1,2)-p(2), po(1,1)-p(1) );
    th2 = atan2( po(2,2)-p(2), po(2,1)-p(1) );
    if (po(2,2)-p(2))*(po(1,1)-p(1)) - (po(2,1)-p(1))*(po(1,2)-p(2)) > 0
        thcl = th2;
        thcr = th1;
    else
        thcl = th1;
        thcr = th2;
    end
end