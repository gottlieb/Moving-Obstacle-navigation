function J = cost_barricade( v, p, thcl, thcr, po, dt )

%% cost_barricade
% cost function of the robot at the location of 'p' and moving with velocity 
% 'v', against barricade at the location of 'po' such that collision cone is
% determined by the two lines 'thcl' and 'thcr'.
%% Inputs
% * v    : the candidate velocity of the robot
% * p    : the location of the robot
% * thcl : the leftmost line enclosing the collision cone
% * thcr : the rightmost line enclosing the collision cone
% * po   : the location of the obstacle
% * dt   : sample time interval in discrete system
%% Outputs
% * J    : the cost
%%   Copyright 
%   Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.5.1.1$  $Date: 2010/11/16$
%

    plmag = 10;
    prmag = 10;
    pl = p+plmag*[cos(thcl),sin(thcl)];
    pr = p+prmag*[cos(thcr),sin(thcr)];
    
    [n,m] = size( v );
    J = zeros( 1, n );
    
    for i=1:n
        absV = v(i,:)-p;
        vmag = sqrt(absV(1)*absV(1) + absV(2)*absV(2));
        
        pf = p + dt*absV;     % final point
        
        th1 = angle_line( po(2,:)-po(1,:), p-po(1,:) );
        th2 = angle_line( po(1,:)-po(2,:), p-po(2,:) );
        if th1 < pi/2 && th2 < pi/2
            d = distptln( pf(1), pf(2), po(1,1), po(1,2), po(2,1), po(2,2) );   % distance from the obstacle
        else
            d = min( norm(p-po(1,:)), norm(p-po(2,:)) );
        end
        
        if (v(i,2)-p(2))*(pl(1)-p(1)) - (v(i,1)-p(1))*(pl(2)-p(2)) < 0 &&...
           (v(i,2)-p(2))*(pr(1)-p(1)) - (v(i,1)-p(1))*(pr(2)-p(2)) > 0
            % v(i) is within VO
            [x,y] = line_intersection( p(1), p(2), pf(1), pf(2), po(1,1), po(1,2), po(2,1), po(2,2) );
            J(i) = -sqrt( absV(1)*absV(1) + absV(2)*absV(2) ) / sqrt( (p(1)-x)^2 + (p(2)-y)^2 );
        else
            dotproduct = (plmag*cos(thcl)*absV(1) + plmag*sin(thcl)*absV(2));
            costheta = dotproduct / (plmag * vmag);
            theta1 = acos(costheta); 
            
            dotproduct = (prmag*cos(thcr)*absV(1) + prmag*sin(thcr)*absV(2));
            costheta = dotproduct / (prmag * vmag);
            theta2 = acos(costheta); 
            
            J(i) = .01*min(theta1,theta2)*vmag/d;
        end
    end
end