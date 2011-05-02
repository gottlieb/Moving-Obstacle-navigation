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