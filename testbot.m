function [pts contacts] = testbot()

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
   
    vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref,...
            [0 0 0], vrep.simx_opmode_oneshot);
        
    pause(10);
   
   
   % Constants:
   
   timestep = .05;
   wheelradius = 0.0937/2; % This value may be inaccurate. Check before using.
   
   % Min max angles for all joints:
   armJointRanges = [-2.9496064186096,2.9496064186096;
                     -1.5707963705063,1.308996796608;
                     -2.2863812446594,2.2863812446594;
                     -1.7802357673645,1.7802357673645;
                     -1.5707963705063,1.5707963705063 ];
   
   startingJoints = [pi,30.91*pi/180,52.42*pi/180,0*pi/180,0];
   
   % In this demo, we move the arm to a preset pose:
   pickupJoints = [180*pi/180, 0.5390, 0.9146, 1.2684, 0*pi/180];
   
   % Tilt of the Rectangle22 box
   r22tilt = -44.56/180*pi;
   
   
   % Parameters for controlling the youBot's wheels:
   forwBackVel = 0;
   leftRightVel = 0;
   rotVel = 0;
   
   disp('Starting robot');
   
%    % Set the arm to its starting configuration:
%    %res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
%    for i = 1:5,
%    [res, j] = vrep.simxGetJointPosition(id, h.armJoints(i), vrep.simx_opmode_oneshot_wait);
%         vrchk(vrep, res, true);
%         j
%    end
   %res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);
   
   plotData = true;
   if plotData,
   subplot(211)
   drawnow;
   [X,Y] = meshgrid(-5:.25:5,-5.5:.25:2.5);
   X = reshape(X, 1, []);
   Y = reshape(Y, 1, []);
   end
   
   mapTest = zeros(8*4*2); %dalladd
   
   % Make sure everything is settled before we start
   pause(2);
   
   [res homeGripperPosition] = ...
     vrep.simxGetObjectPosition(id, h.ptip,...
                                h.armRef,...
                                vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   fsm = 'rotate';
   
  a = false;
  b = false;
  while true,
   tic
   if vrep.simxGetConnectionId(id) == -1,
   error('Lost connection to remote API.');
   end
   
   [res youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
                                                vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   [res youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
                                                     vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   
   vrchk(vrep, res, true);
   [res tipEuler] = vrep.simxGetObjectOrientation(id, h.otip, h.r22,...
                                                     vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   
   % Read data from the Hokuyo sensor:
   [pts contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
   break
   
   %tipEuler
   
   
%    
%    if a
%    T = se2(youbotPos(1), youbotPos(2), youbotEuler(3));
%         p = [-1.325;-5.815;0.2101];
%         p(3) = p(3)-youbotPos(3);
%         p(1:2) = homtrans(inv(T), p(1:2));
%    o = [-pi; 0; 0];
%    
%    vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
%         vrchk(vrep, res, true);
%         
%         vrep.simxSetObjectOrientation(id, h.otarget, h.r22, o,...
%             vrep.simx_opmode_oneshot_wait);
%         vrchk(vrep, res, true);
%         
%         vrep.simxSetObjectPosition(id, h.ptarget, h.ref, p,...
%             vrep.simx_opmode_oneshot_wait);
%         vrchk(vrep, res, true);
%         
%         gripTargDist = 1;
%         while gripTargDist > 0.005
%             [res, tipPos] = vrep.simxGetObjectPosition(id, h.ptip, h.ref,...
%                 vrep.simx_opmode_oneshot_wait);
%             vrchk(vrep, res, true);
%             gripTargDist = sqrt((p(1)-tipPos(1))^2+(p(2)-tipPos(2))^2);
%         end
%   
% %        p(3) = p(3)-youbotPos(3);
% %         vrep.simxSetObjectPosition(id, h.ptarget, h.ref, p,...
% %             vrep.simx_opmode_oneshot_wait);
% %         vrchk(vrep, res, true);
%         
%         a = false;
%    end 
%    
%    if b
%     forwBackVel = -5
%      vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot);
%         vrchk(vrep, res, true);
%         pause(2);
%         b = false
%    end
%    
%    [res, Pos] = vrep.simxGetObjectPosition(id, h.ref, h.armRef,...
%                 vrep.simx_opmode_oneshot_wait);
%             vrchk(vrep, res, true);
%            Pos
%    pause(10);
%    % Update wheel velocities
%    res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
%    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(1),...
%                                    -forwBackVel-leftRightVel+rotVel,...
%                                    vrep.simx_opmode_oneshot); vrchk(vrep, res);
%    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(2),...
%                                    -forwBackVel+leftRightVel+rotVel,...
%                                    vrep.simx_opmode_oneshot); vrchk(vrep, res);
%    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(3),...
%                                    -forwBackVel-leftRightVel-rotVel,...
%                                    vrep.simx_opmode_oneshot); vrchk(vrep, res);
%    vrep.simxSetJointTargetVelocity(id, h.wheelJoints(4),...
%                                    -forwBackVel+leftRightVel-rotVel,...
%                                    vrep.simx_opmode_oneshot); vrchk(vrep, res);
%    res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);
   
   % Make sure that we do not go faster that the simulator
   elapsed = toc;
   timeleft = timestep-elapsed;
   if (timeleft > 0),
   pause(min(timeleft, .01));
   end
   end
   
end % main function
   
