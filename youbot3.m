function youbot3()
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
   
   
   % Make sure everything is settled before we start
   pause(2);
   
   [res homeGripperPosition] = ...
     vrep.simxGetObjectPosition(id, h.ptip,...
                                h.armRef,...
                                vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   fsm = 'rotate';
   
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
   
 
   pose = [youbotPos(1) youbotPos(2) youbotEuler(3)]
  

   forwBackVel = -10;
   leftRightVel = -12;
   rotVel = 5;

   
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
   elapsed = toc;
   timeleft = timestep-elapsed;
   if (timeleft > 0),
   pause(min(timeleft, .01));
   end
   end
   
end % main function
   
