function moving_obstacle_avoidMC( filename )

%% moving_obstacle_avoidMC
% Modified version of Choi's moving_obstacle_avoid() function to support
% Monte Carlo simluations. Mainly strips out all the display functionality.
%
% Requires a filename to be entered, since all filenames will be generated
% by the MC wrapper function.
%% Syntax
% * moving_obstacle_avoid( filename )
%% Description
% * moving_obstacle_avoid( FILENAME ) performs simulation to avoid 
% moving obstacle specified by FILENAME. Does not render simulation
% graphically.
%% Examples
% * moving_obstacle_avoid( 'moving_obstacles.txt' )
%% Inputs
% * filename   : name of the file that specifies moving obstacles. See
% obsread for detail of the specification of the file.
%% References
% [1] Choi, J., <http://users.soe.ucsc.edu/~jwchoi/doc/PhD_Thesis_Choi_Main.pdf Real-Time Obstacle Avoiding Motion Planning for Autonomous
% Ground Vehicles>, University of California, Santa Cruz, 2010.

%% Start of source code: set up parameters

dt = .03;   % robot's discrete time system time interval 
maxSteps = 400;

p = [0,0];  % position of robot
v = [2,1];  % velocity of robot

armax = 4;  % maximum radial acceleration of robot
atmax = 2;  % maximum tangential acceleration of robot

[SOP,MOP,MOV,COP,COV,BR,bndbox] = obsread( filename );     % read obstacle information


%% perform obstacle avoidance
        
m = length(SOP);
n = size(MOP);
n = n(1);
l = size(COP);
l = l(1);
da = 0.1;

for k=1:maxSteps
    theta = atan2( v(end,2), v(end,1) );
    
    vmag = norm(v(end,:));
    theta = atan2( v(end,2), v(end,1) );
    vehAX = [ p(end,1), p(end,1)+(vmag-da)*cos(theta), p(end,1)+(vmag-da)*cos(theta)+da*cos(theta+pi/2),...
             p(end,1)+(vmag)*cos(theta), p(end,1)+(vmag-da)*cos(theta)+da*cos(theta-pi/2), p(end,1)+(vmag-da)*cos(theta) ];
    vehAY = [ p(end,2), p(end,2)+(vmag-da)*sin(theta), p(end,2)+(vmag-da)*sin(theta)+da*sin(theta+pi/2),...
             p(end,2)+(vmag)*sin(theta), p(end,2)+(vmag-da)*sin(theta)+da*sin(theta-pi/2), p(end,2)+(vmag-da)*sin(theta) ];
           
    % max velocity
    vmax = norm(v(end,:))+dt*atmax;
    [cx,cy] = circle( [0,0], vmax, 51 );
    
    % min velocity
    vmin = norm(v(end,:))-dt*atmax;
    [cx,cy] = circle( [0,0], vmin, 51 );
        
    % yaw rate
    omega = armax/norm(v(end,:));
    thl = theta + dt*omega;
    thr = theta - dt*omega;
    [cx,cy] = circularc( [p(end,1) p(end,2) vmax thr thl] );
 
    [vx,vy] = sample_points( p(end,:), vmin, vmax, thl, thr );
    
    % static obstacle
    J = zeros( 1, length(vx) );
    
    for j=1:m
        [thcl,thcr] = findLineCC( p(end,:), SOP{j}(:,1:2) );
        [Js] = cost_static_obs( [vx(:),vy(:)], p(end,:), thcl, thcr, SOP{j}(:,1:2), dt );
        J = J + Js;
    end
    
    % moving obstacle
    for j=1:n
        [ox,oy] = circle( MOP(j,1:2), MOP(j,3), 21 );

        % find absolute Colision Cone
        [thcl,thcr] = findCC( p(end,:), MOP(j,1:2), MOP(j,3) );
        x0 = p(end,1)+MOV(j,1);
        y0 = p(end,2)+MOV(j,2);

        [Js] = cost_moving_obs( [vx(:),vy(:)], p(end,:), thcl, thcr, MOP(j,1:2), MOP(j,3), MOV(j,1:2), dt );
        J = J + 1*Js;
        MOP(j,1) = MOP(j,1) + dt*MOV(j,1);
        MOP(j,2) = MOP(j,2) + dt*MOV(j,2);                
    end
    
    for j=1:l
        % moving obstacles with circular move
        cx = COP(j,1) + COV(j,3)*cos(COV(j,2));
        cy = COP(j,2) + COV(j,3)*sin(COV(j,2));
        [ox,oy] = circle( [cx,cy], COP(j,3), 21 );

        % find absolute Colision Cone
        [thcl,thcr] = findCC( p(end,:), [cx,cy], COP(j,3) );
        x0 = p(end,1) + COV(j,1)*cos(COV(j,2)-pi/2);
        y0 = p(end,2) + COV(j,1)*sin(COV(j,2)-pi/2);

        [Js] = cost_moving_obs( [vx(:),vy(:)], p(end,:), thcl, thcr,...
             [cx,cy], COP(j,3), COV(j,1)*[cos(COV(j,2)-pi/2),sin(COV(j,2)-pi/2)], dt );
        J = J + 1*Js;
        
        COV(j,2) = COV(j,2) - dt*COV(j,1)/COV(j,3);     
        
    end
    
    % leaving barricade
    [thcl,thcr] = findLineCC( p(end,:), BR );
    Jb = cost_barricade( [vx(:),vy(:)], p(end,:), thcl, thcr, BR, dt );
    J = J + .1*Jb;
    
    % calculate new velocity
    ind = find( J==min(J) );
    v = [vx(ind(1))-p(end,1),vy(ind(1))-p(end,2)];
    
    p = [p; p(end,:) + dt*v(end,:)];
    
    if (p(end,2)-BR(1,2))*(BR(2,1)-BR(1,1)) - (p(end,1)-BR(1,1))*(BR(2,2)-BR(1,2)) > 0
        break;
    end    
end

if k >= maxSteps
    error('Time-out');
end

end


%% obsread
function [SOP,MOP,MOV,COP,COV,BR,bndbox] = obsread( filename )     
    % read obstacle information from 'filename'
    % the file consists of entry with length of 6.
    % the set of entries is stored in order of static obstacles (length 4), moving
    % obstacles with linear velocity (length 5), moving obstacles with circular move (length 6),
    % and barricade.
    %
    % outputs 
    % SOP: static obstacles [x0,y0;x1,y1] such that obstacle is a line
    % segment connecting (x0,y0) and (x1,y1).
    % MOP: moving obstacles with linear velocity.
    % [x,y,r], where (x,y), r, are position, radius of the obstacle.
    % MOV: [vx,vy], velocity of the obstacle.
    % COP: moving obstacles with circular move.
    % [x,y,r], where (x,y) is center point of circular trajectory and 
    % r is the radius of the obstacle.
    % COV: [v,theta,R], where v is velocity of the obstacle and R is the 
    % radius of circular trajectory.
    % the initial position of obstalce is at the point determined by theta
    % of the circle.
    % BR: barricade [x0,y0;x1,y1] such that the line connecting (x0,y0) and (x1,y1)
    
    OBS = dlmread( filename );
    n = length( OBS(:,1) );
    SOP = {};
    MOP = [];
    MOV = [];
    COP = [];
    COV = [];
    bndbox = [inf,-inf,inf,-inf];   % bound box for visualization [minX,maxX,minY,maxY]
    
    for i=1:n-1
        if isnan( OBS(i,5) )
            % static obstacle
            SOP = {SOP{:}, [OBS(i,1:2);OBS(i,3:4)] };
            bndbox(1) = min( [bndbox(1), OBS(i,1), OBS(i,3)] );
            bndbox(2) = max( [bndbox(2), OBS(i,1), OBS(i,3)] );
            bndbox(3) = min( [bndbox(3), OBS(i,2), OBS(i,4)] );
            bndbox(4) = max( [bndbox(4), OBS(i,2), OBS(i,4)] );
        elseif isnan( OBS(i,6) )
            % moving obstacles with linear velocity.
            MOP = [MOP; OBS(i,1:3)];
            MOV = [MOV; OBS(i,4:5)];
        else
            % moving obstacles with circular move.
            COP = [COP; OBS(i,1:3)];
            COV = [COV; OBS(i,4:6)];
        end
    end
    BR = [OBS(end,1:2); OBS(end,3:4)];
end