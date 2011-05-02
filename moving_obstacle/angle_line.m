function ANGLE = angle_line( v1, v2 )

%% angle_line
% calculate the angle of the vector, v1->v2.
%% Inputs
% * v1    : the origin of the vector
% * v2    : the tip of the vector
%% Outputs
% * ANGLE : the angle of the vector, v1->v2.
%%   Copyright 
%   Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.5.1.1$  $Date: 2010/11/16$
%

    x1 = v1(1);
    y1 = v1(2);
    x2 = v2(1);
    y2 = v2(2);

    dotproduct = (x1*x2 + y1*y2);

    v1mag = sqrt(x1*x1 + y1*y1);
    v2mag = sqrt(x2*x2 + y2*y2);

    costheta = dotproduct / (v1mag * v2mag);

    ANGLE = acos(costheta); 

end