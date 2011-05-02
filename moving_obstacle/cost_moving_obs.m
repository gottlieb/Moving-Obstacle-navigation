function [J,withinVO] = cost_moving_obs( v, p, thcl, thcr, po, r, vo, dt )

%% cost_moving_obs
% cost function of the robot at the location of 'p' and moving with velocity 
% 'v', against moving obstcle at the location of 'po' with radius 'r' and 
% moving with velocity 'vo' such that collision cone is
% determined by the two lines 'thcl' and 'thcr'.
%% Inputs
% * v    : the candidate velocity of the robot
% * p    : the location of the robot
% * thcl : the leftmost line enclosing the collision cone
% * thcr : the rightmost line enclosing the collision cone
% * po   : the location of the obstacle
% * r    : the radius of the obstacle
% * vo   : the velocity of the obstacle
% * dt   : sample time interval in discrete system
%% Outputs
% * J    : the cost
%%   Copyright 
%   Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.5.1.1$  $Date: 2010/11/16$
%

    plmag = 10;
    prmag = 10;
    relpl = p+plmag*[cos(thcl),sin(thcl)];
    relpr = p+prmag*[cos(thcr),sin(thcr)]; 
            
    [n,m] = size( v );
    J = zeros( 1, n );

    withinVO = ones(1,n);  % 1 if i-th candidate velocity is within VO
    
    for i=1:n
        absV = v(i,:)-p;
        vmag = sqrt(absV(1)*absV(1) + absV(2)*absV(2));
        
        relv = absV-vo;
        pf = p + dt*(relv);     % final relative point
        
        dobs = norm( pf-po ) - r;      % distance from the obstacle
        if dobs < r
            J(i) = Inf;
        else
        
            if (relv(2))*(relpl(1)-p(1)) - (relv(1))*(relpl(2)-p(2)) < 0 && (relv(2))*(relpr(1)-p(1)) - (relv(1))*(relpr(2)-p(2)) > 0
                % v(i) is within VO

                % intersection between p->pf and the obstacle circle
                thpf = atan2(pf(2)-p(2),pf(1)-p(1));
                thpo = atan2(po(2)-p(2),po(1)-p(1));
                phi = mod( thpf - thpo, 2*pi );
                d = norm( po-p );
                theta = pi/2 - abs(phi) - acos( d*sin(abs(phi))/r );
                if phi > 0
                    intp = po + r*[cos(thpo+pi-theta),sin(thpo+pi-theta)];
                else
                    intp = po + r*[cos(thpo+pi+theta),sin(thpo+pi+theta)];
                end
                J(i) = norm(relv) / norm( pf-intp );      % inverse of collision time
            else
                withinVO(i) = 0;
                
                dotproduct = (plmag*cos(thcl)*absV(1) + plmag*sin(thcl)*absV(2));
                costheta = dotproduct / (plmag * vmag);
                theta1 = acos(costheta); 

                dotproduct = (prmag*cos(thcr)*absV(1) + prmag*sin(thcr)*absV(2));
                costheta = dotproduct / (prmag * vmag);
                theta2 = acos(costheta); 

                J(i) = -.01*min(theta1,theta2)*vmag/dobs;      
            end
        
        end
    end
end