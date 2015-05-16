function youbot_exploration()
% youbot Illustrates the V-REP Matlab bindings.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
   
   disp('Program started');
   %Use the following line if you had to recompile remoteApi
   %vrep = remApi('remoteApi', 'extApi.h');
   vrep=remApi('remoteApi');
   vrep.simxFinish(-1);
   id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
   
   if id < 0,
       disp('Failed connecting to remote API server. Exiting.');
       vrep.delete();
       return;
   end
   fprintf('Connection %d to remote API server open.\n', id);
   
   % Make sure we close the connexion whenever the script is interrupted.
   cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
   
   % This will only work in "continuous remote API server service"
   % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
   res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
   % We're not checking the error code - if vrep is not run in continuous remote
   % mode, simxStartSimulation could return an error.
   % vrchk(vrep, res);
   
   % Retrieve all handles, and stream arm and wheel joints, the robot's pose,
   % the Hokuyo, and the arm tip pose.
   h = youbot_init(vrep, id);
   h = youbot_hokuyo_init(vrep, h);
   
   % Let a few cycles pass to make sure there's a value waiting for us next time
   % we try to get a joint angle or the robot pose with the simx_opmode_buffer
   % option.
   pause(.2);
   
   % Constants:
   
   timestep = .05;
   wheelradius = 0.0937/2; % This value may be inaccurate. Check before using.
   
   % Min max angles for all joints:
   armJointRanges = [-2.9496064186096,2.9496064186096;
                     -1.5707963705063,1.308996796608;
                     -2.2863812446594,2.2863812446594;
                     -1.7802357673645,1.7802357673645;
                     -1.5707963705063,1.5707963705063 ];
   
   startingJoints = [0,30.91*pi/180,52.42*pi/180,72.68*pi/180,0];
   
   % In this demo, we move the arm to a preset pose:
   pickupJoints = [90*pi/180, 19.6*pi/180, 113*pi/180, -41*pi/180, 0*pi/180];
   
   % Tilt of the Rectangle22 box
   r22tilt = -44.56/180*pi;
   
   
   % Parameters for controlling the youBot's wheels:
   forwBackVel = 0;
   leftRightVel = 0;
   rotVel = 0;
   
   %Trajectory test
%    points = [-4 6.2; -0.5 6.2; -0.5 5; 6.2 5; 6.2 4.5];
   index = 1;
   
   disp('Starting robot');
   
   % Set the arm to its starting configuration:
   res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
   for i = 1:5,
       res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                                             startingJoints(i),...
                                             vrep.simx_opmode_oneshot);
       vrchk(vrep, res, true);
   end
   res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);
   
   d = 7.5;
   cell_size = 0.25;
   
   plotData = true;
   if plotData,
       %subplot(211)
       %drawnow;
       [X,Y] = meshgrid(-5:cell_size:5,-5.5:cell_size:2.5);
       X = reshape(X, 1, []);
       Y = reshape(Y, 1, []);
   end
   
   colormap([0 104/255 139/255; 0 1 127/255; 205/255 38/255 38/255; 0 0 0; 1 0.5 0]);
   map = zeros((d/cell_size)*2); map(:) = -1;%ajout
   
   % Make sure everything is settled before we start
   pause(2);
   
   [res homeGripperPosition] = ...
     vrep.simxGetObjectPosition(id, h.ptip,...
                                h.armRef,...
                                vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   fsm = 'rotate';
   [res youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
                                                    vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   [res youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
                                                         vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
                                                
   initial_rotation = 1;
   traj_timer = tic;
   elapsed_time = 0; % elapsed time since the begining  of a trajectory
   next_point_delay = 1; % time interval before selecting the next pursuit point
   new_traject = 1;
   d_star = 0.3; % The robot maintains this distance behind a pursuit point
   prev_e = 0.5;
   traj_indices = [-1, -1];
   fill_point = [-1 -1];
   new_point = 1;
   x_init = 0;
   y_init = 0;
   precision = 0;
   
   while true,
       sim_timer = tic;
       if vrep.simxGetConnectionId(id) == -1,
          error('Lost connection to remote API.');
       end

       [res youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
                                                    vrep.simx_opmode_buffer);
       vrchk(vrep, res, true);
       [res youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
                                                         vrep.simx_opmode_buffer);
       vrchk(vrep, res, true);

       if plotData,
           % Read data from the Hokuyo sensor:
           [pts contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);

           in = inpolygon(X, Y, [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)],...
                          [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)]);

%            subplot(211)
%            plot(X(in), Y(in),'.g', pts(1,contacts), pts(2,contacts), '*r',...
%                 [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)],...
%                 [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)], 'r',...
%                 0, 0, 'ob',...
%                 h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or',...
%                 h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or');
%            axis([-5.5 5.5 -5.5 2.5]);
%            axis equal;
%            drawnow;

       end
       
       pose = [youbotPos(1) youbotPos(2) youbotEuler(3)];
       a = floor((pose(1)+d)/cell_size)+1;
       b = floor((pose(2)+d)/cell_size)+1;
       
       if(fill_point(1) == -1)
           fill_point = [a b];
       end  

       % Mark the robot's position as observed
       if map(a, b) ~= 1
           map(a, b) = 0;
       end
       if map(a+1, b) ~= 1
           map(a+1, b) = 0;
       end
       if map(a, b+1) ~= 1
           map(a, b+1) = 0;
       end
       if map(a+1, b+1) ~= 1
           map(a+1, b+1) = 0;
       end
       if map(a+1, b-1) ~= 1
           map(a+1, b-1) = 0;
       end
       if map(a-1, b+1) ~= 1
           map(a-1, b+1) = 0;
       end
       if map(a-1, b-1) ~= 1
           map(a-1, b-1) = 0;
       end
       if map(a-1, b) ~= 1
           map(a-1, b) = 0;
       end
       if map(a, b-1) ~= 1
           map(a, b-1) = 0;
       end
       
       % update the map
       map = ptsToCellmap(X(in), Y(in), pts(1,contacts), pts(2,contacts), map,...
                cell_size,pose,d,true);
       
       trajmap = map;
       if traj_indices(1) ~= -1
           trajmap(sub2ind(size(map),traj_indices(:,1),traj_indices(:,2))) = 2;
       end
       trajmap(a,b) = 3;
       imagesc(trajmap);
       drawnow;
       
       if initial_rotation == 1 % Complete rotation at start
           rotVel = 5;
           if toc(traj_timer) > 6,
               rotVel = 0;
               initial_rotation = 0;
           end
            
       else % The rotation at start has been performed
        
           if new_traject == 1 % Destination reached -> Need to plan the next move
                [traj, map, traj_indices] = planNextMove(map, pose, d, cell_size, fill_point);
                traj
                if isempty(traj)
                    break;
                end
                s = size(traj);
                index = 1;
                new_traject = 0;
                traj_timer = tic;
                prev_t = 0;
           end
           
           trajmap = map;
           trajmap(sub2ind(size(map),traj_indices(:,1),traj_indices(:,2))) = 2;
           trajmap(a,b) = 3;
           imagesc(trajmap);
           drawnow;
           

           x = pose(1);
           x_star = traj(index,1);
           y = pose(2);
           y_star = traj(index,2);
           theta = pose(3);
           theta_star = atan2((y_star - y),(x_star - x))+pi/2;

           % Check if we are not running to an obstacle
           if index == s(1) || map(traj_indices(index+1,1),traj_indices(index+1,2)) == 1
                 new_traject = 1;
           end

           t = toc(traj_timer);
           e = sqrt((x_star-x)^2+(y_star-y)^2)-d_star;
           if e > 0.01
               v_star = 20*e + 30*(abs(t-prev_t)*abs(e-prev_e)/2);
               alpha = 15*angdiff(theta_star, theta);
               gamma = -theta+atan2((y_star - y),(x_star - x));
               forwBackVel = v_star*sin(gamma);
               leftRightVel = v_star*cos(gamma);
               rotVel = alpha;
               
           else
              index = index + 1;
              if index == s(1)
                  new_traject = 1;
                  forwBackVel = 0;
                  leftRightVel = 0;
                  rotVel = 0;
              end
           end
 
       end

       % Update wheel velocities
       res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
       vrep.simxSetJointTargetVelocity(id, h.wheelJoints(1),...
                                       -forwBackVel-leftRightVel+rotVel,...
                                       vrep.simx_opmode_oneshot); vrchk(vrep, res);
       vrep.simxSetJointTargetVelocity(id, h.wheelJoints(2),...
                                       -forwBackVel+leftRightVel+rotVel,...
                                       vrep.simx_opmode_oneshot); vrchk(vrep, res);
       vrep.simxSetJointTargetVelocity(id, h.wheelJoints(3),...
                                       -forwBackVel-leftRightVel-rotVel,...
                                       vrep.simx_opmode_oneshot); vrchk(vrep, res);
       vrep.simxSetJointTargetVelocity(id, h.wheelJoints(4),...
                                       -forwBackVel+leftRightVel-rotVel,...
                                       vrep.simx_opmode_oneshot); vrchk(vrep, res);
       res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);

       % Make sure that we do not go faster that the simulator
       elapsed = toc(sim_timer);
       timeleft = timestep-elapsed;
       if (timeleft > 0),
       pause(min(timeleft, .01));
       end
   end
   
end % main function
   
