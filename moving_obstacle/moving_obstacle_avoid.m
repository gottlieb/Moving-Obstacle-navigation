function moving_obstacle_avoid( filename, moviename )

%% moving_obstacle_avoid
% Main program for moving obstacle avoidance simulation.
% The simulation applies the moving obstacle avoidance algorithm proposed
% in Choi's thesis [1].
%% Syntax
% * moving_obstacle_avoid( filename )
% * moving_obstacle_avoid( filename, moviename )
%% Description
% * moving_obstacle_avoid( FILENAME ) performs simulation to avoid 
% moving obstacle specified by FILENAME.
% * moving_obstacle_avoid( FILENAME, MOVIENAME ) performs simulation to avoid 
% moving obstacle specified by FILENAME and saves the animation into movie of which name is MOVIENAME.
%% Examples
% * moving_obstacle_avoid( 'moving_obstacles.txt' )
% * moving_obstacle_avoid( 'moving_obstacles.txt', 'moving_obstacle_avoid.avi' )
%% Inputs
% * filename   : name of the file that specifies moving obstacles. See
% obsread for detail of the specification of the file.
% * moviename  : name of movie file
%% References
% [1] Choi, J., <http://users.soe.ucsc.edu/~jwchoi/doc/PhD_Thesis_Choi_Main.pdf Real-Time Obstacle Avoiding Motion Planning for Autonomous
% Ground Vehicles>, University of California, Santa Cruz, 2010.
%%   Copyright 
%   Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)
%   $Revision: 1.5.1.1$  $Date: 2010/11/16$
%

%% Start of source code: set up parameters

% addpath(genpath('../common'));  % add common directory to the search path

if nargin < 1
    filename = 'moving_obstacles.txt';
end

% drawmode to indicate drawing mode: 1-saves the animation into movie,
% 0-otherwise.
if nargin > 1
    drawmode = 1;
else
    drawmode = 0;
end

verbose = 0;    % debuge mode

dt = .03;   % robot's discrete time system time interval 

p = [0,0];  % position of robot
v = [2,1];  % velocity of robot

armax = 4;  % maximum radial acceleration of robot
atmax = 2;  % maximum tangential acceleration of robot

[SOP,MOP,MOV,COP,COV,BR,bndbox] = obsread( filename );     % read obstacle information

%% visualize
fillcolor = [0,.9,0];   % Color for collision cone
edgecolor = [0,.5,0];   % Collision cone edge color
facealpha = .2;
L = 10;     % length of collision cone line segment to visualize moving obstacle velocity

% create figure window
delete(gcf);
fig = figure();
if ~drawmode
    units=get(fig,'units');
    set(fig,'units','normalized','outerposition',[0 0 .7 .7]);
    set(fig,'units',units);
end
set( gca, 'xtick', [] );
set( gca, 'ytick', [] );
daspect( [1 1 1] );
box on;
hold on;
axis( bndbox + [-1,1,-1,0] );

n = length(SOP);
for i=1:n
    % visualize static obstacles
    plot( SOP{i}(:,1), SOP{i}(:,2), 'k', 'LineWidth', 5 );
    
    % find absolute Colision Cone
    [thcl,thcr] = findLineCC( p(end,:), SOP{i}(:,1:2) );
    hcs(i) = fill( [p(1),p(1)+L*cos(thcl),p(1)+L*cos(thcr),p(1)],...
                   [p(2),p(2)+L*sin(thcl),p(2)+L*sin(thcr),p(2)],...
               fillcolor, 'FaceAlpha', facealpha,'EdgeColor',edgecolor );
    set(hcs(i),'Erasemode','normal');
end

n = size(MOP);
n = n(1);
for i=1:n
    % moving obstacles
    [ox,oy] = circle( [0, 0], MOP(i,3), 21 );
    ho(i) = fill( ox+MOP(i,1), oy+MOP(i,2), 'k' );  % Draw straight-line obstacles
    set(ho(i),'Erasemode','normal');
    
    % find absolute Colision Cone
    [thcl,thcr] = findCC( p(end,:), MOP(i,1:2), MOP(i,3) );
    x0 = p(1)+MOV(i,1);
    y0 = p(2)+MOV(i,2);
    hc(i) = fill( [x0,x0+L*cos(thcl),x0+L*cos(thcr),x0],...
               [y0,y0+L*sin(thcl),y0+L*sin(thcr),y0],...
               fillcolor, 'FaceAlpha', facealpha,'EdgeColor',edgecolor ); % Draw straight-line collision cones
    set(hc(i),'Erasemode','normal');
end

n = size(COP);
n = n(1);
for i=1:n
    % moving obstacles with circular move
    [ox,oy] = circle( [0, 0], COP(i,3), 21 );
    hoc(i) = fill( ox+COP(i,1) + COV(i,3)*cos(COV(i,2)), oy+COP(i,2) + COV(i,3)*sin(COV(i,2)), 'k' ); % draw circular movement obstacles
    set(hoc(i),'Erasemode','normal');
    
    % find absolute Colision Cone
    [thcl,thcr] = findCC( p(end,:), COP(i,1:2)+COV(i,3)*[cos(COV(i,2)),sin(COV(i,2))], COP(i,3) );
    x0 = p(1) + COV(i,1)*cos(COV(i,2)-pi/2);
    y0 = p(2) + COV(i,1)*sin(COV(i,2)-pi/2);
    hcc(i) = fill( [x0,x0+L*cos(thcl),x0+L*cos(thcr),x0],...
               [y0,y0+L*sin(thcl),y0+L*sin(thcr),y0],...
               fillcolor, 'FaceAlpha', facealpha,'EdgeColor',edgecolor ); % Draw circular movement collision cones
    set(hcc(i),'Erasemode','normal');
end

vehX = .2*[-.9, -.7, -.7, -.3, -.3, .3, .3, .7, .7, .9,  1,...
                  1, .9, .7, .7, .3, .3, -.3, -.3, -.7, -.7, -.9, -.9 ];
vehY = .2*[ .5,  .5,  .4,  .4,  .5, .5, .4, .4, .5, .5, .4,... 
        -.4, -.5, -.5, -.4, -.4, -.5, -.5, -.4, -.4, -.5, -.5, .5 ];
hv = fill( vehX, vehY, [.5,1,1] );
set(hv,'Erasemode','normal');

% velocity arrow for robot
da = .1;
vmag = norm(v);
theta = atan2( v(2), v(1) );
vehAX = [  p(1), p(1)+(vmag-da)*cos(theta), p(1)+(vmag-da)*cos(theta)+da*cos(theta+pi/2),...
          p(1)+(vmag)*cos(theta), p(1)+(vmag-da)*cos(theta)+da*cos(theta-pi/2), p(1)+(vmag-da)*cos(theta) ];
vehAY = [  p(2), p(2)+(vmag-da)*sin(theta), p(2)+(vmag-da)*sin(theta)+da*sin(theta+pi/2),...
          p(2)+(vmag)*sin(theta), p(2)+(vmag-da)*sin(theta)+da*sin(theta-pi/2), p(2)+(vmag-da)*sin(theta) ];
hva = fill( vehAX, vehAY, 'k' );
set(hva,'Erasemode','normal');

% max velocity circle
vmax = norm(v)+dt*atmax;
[cx,cy] = circle( p, vmax, 51 );
hvmax = plot( cx,cy, 'k' );
set(hvmax,'Erasemode','normal');

% min velocity circle
vmin = norm(v)-dt*atmax;
[cx,cy] = circle( p, vmin, 51 );
hvmin = plot( cx,cy, 'k' );
set(hvmin,'Erasemode','normal');

% yaw rate
omega = armax/norm(v);
thl = theta + dt*omega;
thr = theta - dt*omega;
[cx,cy] = circularc( [p(end,1) p(end,2) vmax thr thl] );
hy = plot( [p(1),cx,p(1)], [p(2),cy,p(2)], 'k' );
set(hy,'Erasemode','normal');

if drawmode
    aviobj = avifile(moviename, 'compression', 'Indeo5');
    Frames = getframe(gcf);
    aviobj = addframe(aviobj,Frames);
end

%% perform obstacle avoidance
        
m = length(SOP);
n = size(MOP);
n = n(1);
l = size(COP);
l = l(1);
for k=1:400
    theta = atan2( v(end,2), v(end,1) );
    set(hv,'Xdata',vehX*cos(theta)-vehY*sin(theta)+p(end,1),...
           'Ydata',vehY*cos(theta)+vehX*sin(theta)+p(end,2));
    if verbose && mod(k,2)==1
        plot( vehX*cos(theta)-vehY*sin(theta)+p(end,1), vehY*cos(theta)+vehX*sin(theta)+p(end,2) );
    end
    
    vmag = norm(v(end,:));
    theta = atan2( v(end,2), v(end,1) );
    vehAX = [ p(end,1), p(end,1)+(vmag-da)*cos(theta), p(end,1)+(vmag-da)*cos(theta)+da*cos(theta+pi/2),...
             p(end,1)+(vmag)*cos(theta), p(end,1)+(vmag-da)*cos(theta)+da*cos(theta-pi/2), p(end,1)+(vmag-da)*cos(theta) ];
    vehAY = [ p(end,2), p(end,2)+(vmag-da)*sin(theta), p(end,2)+(vmag-da)*sin(theta)+da*sin(theta+pi/2),...
             p(end,2)+(vmag)*sin(theta), p(end,2)+(vmag-da)*sin(theta)+da*sin(theta-pi/2), p(end,2)+(vmag-da)*sin(theta) ];
       
    % velocity arrow
    set(hva,'Xdata',vehAX, 'Ydata',vehAY );
    
    % max velocity
    vmax = norm(v(end,:))+dt*atmax;
    [cx,cy] = circle( [0,0], vmax, 51 );
    set(hvmax,'Xdata',cx+p(end,1), 'Ydata',cy+p(end,2));
    
    % min velocity
    vmin = norm(v(end,:))-dt*atmax;
    [cx,cy] = circle( [0,0], vmin, 51 );
    set(hvmin,'Xdata',cx+p(end,1), 'Ydata',cy+p(end,2));
        
    % yaw rate
    omega = armax/norm(v(end,:));
    thl = theta + dt*omega;
    thr = theta - dt*omega;
    [cx,cy] = circularc( [p(end,1) p(end,2) vmax thr thl] );
    set(hy,'Xdata',[p(end,1),cx,p(end,1)], 'Ydata', [p(end,2),cy,p(end,2)] );

    [vx,vy] = sample_points( p(end,:), vmin, vmax, thl, thr );
    
    % static obstacle
    J = zeros( 1, length(vx) );
    
    for j=1:m
        [thcl,thcr] = findLineCC( p(end,:), SOP{j}(:,1:2) );
        set( hcs(j), 'Xdata', [p(end,1),p(end,1)+L*cos(thcl),p(end,1)+L*cos(thcr),p(end,1)],...
                     'Ydata', [p(end,2),p(end,2)+L*sin(thcl),p(end,2)+L*sin(thcr),p(end,2)] );
        [Js] = cost_static_obs( [vx(:),vy(:)], p(end,:), thcl, thcr, SOP{j}(:,1:2), dt );
        J = J + Js;
    end
    
    % moving obstacle
    for j=1:n
        [ox,oy] = circle( MOP(j,1:2), MOP(j,3), 21 );
        set( ho(j),'Xdata',ox, 'Ydata',oy );
        if verbose && mod(k,2)==1
            plot( ox, oy, 'k' );
        end
        % find absolute Colision Cone
        [thcl,thcr] = findCC( p(end,:), MOP(j,1:2), MOP(j,3) );
        x0 = p(end,1)+MOV(j,1);
        y0 = p(end,2)+MOV(j,2);
        set( hc(j), 'Xdata', [x0,x0+L*cos(thcl),x0+L*cos(thcr),x0],...
                    'Ydata', [y0,y0+L*sin(thcl),y0+L*sin(thcr),y0] );

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
        set( hoc(j),'Xdata',ox, 'Ydata',oy );
        if verbose && mod(k,2)==1
            plot( ox, oy, 'k' );
        end
        % find absolute Colision Cone
        [thcl,thcr] = findCC( p(end,:), [cx,cy], COP(j,3) );
        x0 = p(end,1) + COV(j,1)*cos(COV(j,2)-pi/2);
        y0 = p(end,2) + COV(j,1)*sin(COV(j,2)-pi/2);
        set( hcc(j), 'Xdata', [x0,x0+L*cos(thcl),x0+L*cos(thcr),x0],...
                    'Ydata', [y0,y0+L*sin(thcl),y0+L*sin(thcr),y0] );

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
    if drawmode
        Frames = getframe(gcf);
        aviobj = addframe(aviobj,Frames);
    else
        pause(.1);
    end 
    
end


if drawmode
    Frames = getframe(gcf);
    aviobj = addframe(aviobj,Frames);

    close(gcf)
    aviobj = close(aviobj);
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