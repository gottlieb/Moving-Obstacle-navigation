% MC(tests)
%
% This function runs a Monte Carlo simulation of Choi's velocity-obstacle
% demo.
% 
% The parameter 'tests' designates the number of simulations to be
% performed for each case. The number of cases is....
%
% The simulation utilizes a 12x12 environment (in Matlab units) bounded on
% three sides by rectangular static obstacles. It then cycles through the
% obstacle densities* from 0-75% in 5% increments. At each increment, it
% apportions the total density among the three obstacle class**, also in 5%
% increments. Once the apportionment is done, it is further subdivided
% based on the number of obstacles in each class, where the number varies
% based on a minimum radius of 0.2 and a maximum radius of 2, with steps of
% 0.2 in radius size. Within each subdivision, all of the obstacles
% belonging to the same class are the same size. Obstacles in different
% classes may be different sizes.
%
% Once the obstacle parameters are established for a given test, an
% obstacle description file is created and the simulation is run on that
% file. This version of the simulation eliminates the graphical
% presentation that Choi's version contained.
%
% Choi's demo generates an error if the robot collides with an object.
% Otherwise it exits normally. This function catches the error and
% increments its failure count if there is a collision. Otherwise it
% increments its success count.
%
% Finally, after each test, a line describing the exact parameters of the
% obstacles in the test is output to the file named in "outfile" as well as
% an indicator of success or failure to be used for later analysis.
%
% Written by Jeremy Gottlieb, Autonomous Systems Lab, UCSC
% Last Modified: 4/22/2011

function MC(tests, outfile)

    success = 0;
    fail = 0;

    [filenames, params] = makeObsFiles(.75, tests);
%             try
%                 moving_obstacle_avoidMC(filename);
%                 success = success + 1;
%             catch
%                 fail = fail+1;
%             end
%     disp(['Success percentage = '])
%     disp((success/i) * 100)
end


% makeObsFiles(maxDensity, tests) generates the obstacle files and returns a
% list of filenames. 
% maxDensity is the maximum coverage of 
% params is the specific parameters encoded
% in the corresponding file.

function [filenames, params] = makeObsFiles(maxDensity, tests)
    totalArea = 144;
    maxArea = totalArea * maxDensity;
    maxObsR = 2.0;
    minObsR = 0.2;
    files = 0;
    
    % The boundaries are rectangular static obstacles that will go at the
    % top of every obstacle file.
    Bounds = sprintf('-4,0,-4,12,nan,nan\n8,0,8,12,nan,nan\n-4,-0.1,8,-0.1,nan,nan\n');
    
    % The goal is a horizontal barrier that will go at the end of every
    % obstacle file
    Goal = sprintf('-4,12, 8,12,nan,nan');
   
    % First create the input files for no obstacles
    for k=1:tests
        filename = sprintf('obstacles/%03d_00_00_00_00_00_00.txt', k);
        fid = fopen(filename, 'w');
        fprintf(fid, '%s', Bounds);
        fprintf(fid, '%s', Goal);
        files = files + 1;
        filenames(files,:) = filename;
        params(files,:) = [0 0 0 0 0 0 0];
        fclose(fid);
    end
    
    % Then create the input files where there are only obstacles of one
    % type.
    % To simplify this, all obstacles will be the same size. So we will
    % step through the sizes from minObsR to maxObsR in increments of 0.2.
    % For each obstacle size, we will step through the possible number of
    % obstacles, adding them at random locations and, where appropriate,
    % with random velocities.
    for i = .2:.2:2
        maxObs = min(99, ceil(totalArea*maxDensity/(i^2 * pi)));
        for j=1:maxObs
            density = ((i^2*pi)*j)/totalArea;
            for k=1:tests % Tests with just static obstacles
                filename = sprintf('obstacles/%03d_%02d_%02d_00_00_00_00.txt', k, cast(density, 'uint16'), j);
                [fid, message] = fopen(filename, 'w');
                fprintf(fid, '%s', Bounds);
                for m=1:j
                    x = (rand * 12) - 4;
                    y = (rand * 12) - 4;
                    fprintf(fid, '%.1f,%.1f,%.1f,0,0,nan\n', x, y, i);
                end
                fprintf(fid, '%s', Goal);
                files = files + 1;
                filenames(files,:) = filename;
                params(files,:) = [density density j 0 0 0 0];
                fclose(fid);
            end
        end
    end
end