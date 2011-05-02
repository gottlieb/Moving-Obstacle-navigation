function [thcl,thcr] = findCC( p, po, r )

%% findCC
% Find the two lines to determine absolute collision cone of the robot with the
% location 'p' relative to the obstacle with the location 'po' and radius
% 'r'.
%% Inputs
% * p   : the location of the robot
% * po  : the location of the obstacle
% * r   : the radius of the obstacle
%% Outputs
% * thcl : angle of the left line enclosing the collision cone
% * thcr : angle of the right line enclosing the collision cone
%%   Copyright 
%   Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.5.1.1$  $Date: 2010/11/16$
%

    d = norm( po - p );
    th = atan2( po(2)-p(2), po(1)-p(1) );
    eta = asin( r/d );
    if ~isreal(th)
        error('Imaginary angle calculated');
    end
    if ~isreal(eta)
        error('Imaginary angle calculated');
    end
    thcl = th + eta;
    thcr = th - eta;
end