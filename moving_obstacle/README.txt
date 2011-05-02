This package implements moving obstacle avoidance simulation using the moving obstacle avoidance algorithm proposed
in Choi's thesis [1].

EXAMPLE: Under the working directory, type the following in Matlab command window.

>> moving_obstacle_avoid( 'moving_obstacles.txt' )

The package is categorized into source code, obstacle data file, help file.

The source code is broken down into:

* main program
 - moving_obstacle_avoid.m
* moving obstacle avoidance calculation
 - cost_barricade.m
 - cost_moving_obs.m
 - cost_static_obs.m
 - findCC.m
 - findLineCC.m
 - sample_points.m
% graphic function
 - angle_line.m
 - circle.m
 - circularc.m
 - distptln.m
 - line_intersection.m

The obstacle data file is "moving_obstacles.txt."
The file consists of entry with length of 6.
The set of entries is stored in order of static obstacles (length 4), moving obstacles with linear velocity (length 5), 
moving obstacles with circular move (length 6), and barricade.

"html" folder contains help files of each source code in the form of html.


% References
[1] Choi, J., Real-Time Obstacle Avoiding Motion Planning for Autonomous Ground Vehicles, 
University of California, Santa Cruz, 2010.
http://users.soe.ucsc.edu/~jwchoi/doc/PhD_Thesis_Choi_Main.pdf

% Copyright 
Ji-Wung Karl Choi (jwchoi@soe.ucsc.edu)