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

function MC(tests, maxpercent, outfile)

    success = [];
    fail = [];

    files = [];
    stats = zeros(1, 6);
    
    for i=1:floor(maxpercent/5)
        perms = apportion(i); % Generates all possible permutations of density assignment
        [a, b] = size(perms); % Figures out how many permutations were generated
        for j =1:a
            [filenames, params] = makeObsFiles(perms(j,:), tests); % Generate random obstacles
            files = cat(1, files, filenames); % Add filenames to list
            stats = cat(1, stats, params); % Add statistics about these tests to list
            for f = 1:length(filenames)
                try
                    moving_obstacle_avoidMC(filenames(f,:)); % Run test
                    success = cat(1, success, 1); % Tally successes
                catch
                    success = cat(1, success, 0);
                end
            end
            save(outfile, 'files', 'stats'); % Save after every permutation for benchmarking
            disp(perms(j,:))
        end
    end
    stats(1,:) = [];
    disp(['Overall success percentage = '])
    disp((sum(success)/length(stats(:,1))) * 100)
    save outfile success -append
end


% makeObsFiles(maxDensity, tests) generates the obstacle files and returns a
% list of filenames. 
% maxDensity is the maximum coverage of 
% params is the specific parameters encoded
% in the corresponding file.

function [filenames, params] = makeObsFiles(perm, tests)
    totalArea = 144;
    maxObsR = 2.0;
    minObsR = 0.2;
    maxObs = 99;
    files = 0;
    
    % The boundaries are rectangular static obstacles that will go at the
    % top of every obstacle file.
    Bounds = sprintf('-4,0,-4,12,nan,nan\n8,0,8,12,nan,nan\n-4,-0.1,8,-0.1,nan,nan\n');
    
    % The goal is a horizontal barrier that will go at the end of every
    % obstacle file
    Goal = sprintf('-4,12, 8,12,nan,nan');
   
    % To simplify matters, all obstacles will be of the same size. Simply
    % go through each slot of the given permutation and generate the
    % appropriate number of obstacles for a given obstacle size. Do this
    % for all possible sizes.
    for i = minObsR:.2:maxObsR
        for k=1:tests
            % First figure out how many of each obstacle is required.
            % Capped at 99 obstacles of any type because for smaller
            % objects there just get to be too many.
            numStatic = min(maxObs, round((perm(1) * .05 * totalArea)/(pi * i^2)));
            numLinear = min(maxObs, round((perm(2) * .05 * totalArea)/(pi * i^2)));
            numCircle = min(maxObs, round((perm(3) * .05 * totalArea)/(pi * i^2)));
            % filname format contains the permutation and the number of
            % obstacles of each type.
            filename = sprintf('obstacles/%02d_%02d_%02d_%02d_%02d_%02d_%03d.txt', ...
                perm(1)*5, numStatic, perm(2)*5, numLinear, perm(3)*5, numCircle, k);
            [fid, message] = fopen(filename, 'w');
            fprintf(fid, '%s', Bounds);
            
            % Generate static obstacles. These are formatted as linear
            % velocity obstacles with 0 velocity.
            for m=1:numStatic
                x = (rand * 12) - 4; % Range [-4,8]
                y = (rand * 12);     % Range [0, 12]
                fprintf(fid, '%.1f,%.1f,%.1f,0,0,nan\n', x, y, i);
            end
            
            % Generate obstacles with linear velocity. x and y velocities
            % come from random interval [0, 2]
            for m=1:numLinear
                x = (rand * 12) - 4; % Range [-4,8]
                y = rand * 12;     % Range [0, 12]
                vx = rand * 2 * sign(randn); % Range [-2, 2]
                vy = rand * 2 * sign(randn); % Range [-2, 2]
                fprintf(fid, '%.1f,%.1f,%.1f,%.1f,%.1f,nan\n', x, y, i, vx, vy);
            end
            
            % Generate obstacles with circular velocity.
            % center of velocity circle also in range of field.
            % Velocity radius random on the range [0, 5]
            for m=1:numCircle
                x = (rand * 12) - 4; % Range [-4,8]
                y = rand * 12;     % Range [0, 12]
                vx = (rand * 12) - 4; % Range [-4,8]
                vy = rand * 12;     % Range [0, 12]
                vr = rand * 5;  % Range [0, 5]
                fprintf(fid, '%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n', x, y, i, vx, vy, vr);
            end

            fprintf(fid, '%s', Goal);
            files = files + 1;
            filenames(files,:) = filename;
            
            % Generate a list of parameters corresponding to the
            % appropriate filename for later data analysis.
            StaticDens = numStatic*(pi*i^2)/totalArea;
            LinearDens = numLinear*(pi*i^2)/totalArea;
            CircleDens = numCircle*(pi*i^2)/totalArea;
            params(files,:) = [StaticDens numStatic LinearDens numLinear CircleDens numCircle];
            fclose(fid);
        end
    end
end