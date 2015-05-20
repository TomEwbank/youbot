function youbot_B1D2()
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
d_star = 0.2; % The robot maintains this distance behind a pursuit point
c_box = 0.05;
d_cyl = 0.05;
d_table = 0.8;
h_table = 0.185;
d_basket = d_table;
h_basket = 0.185;


% Min max angles for all joints:
armJointRanges = [-2.9496064186096,2.9496064186096;
    -1.5707963705063,1.308996796608;
    -2.2863812446594,2.2863812446594;
    -1.7802357673645,1.7802357673645;
    -1.5707963705063,1.5707963705063 ];

startingJoints = [0,30.91*pi/180,52.42*pi/180,72.68*pi/180,0];


% Parameters for controlling the youBot's wheels:
forwBackVel = 0;
leftRightVel = 0;
rotVel = 0;

% table positions
table1 = [-3 -6];
r_table_traj = d_table/2+0.25;
n_table_traj = 200;
table1_traj = circle(table1, r_table_traj, 'n', n_table_traj);
r_table_zone = d_table/2+0.4;
n_table_zone = 15;
table1_zone = circle(table1, r_table_zone, 'n', n_table_zone);
startRound = true;
startGoing = true;
round = 0;
objectPickedUp = false;

% instructions
inst = struct('shape', 'box', 'pose', [-3.275; -6.15; 0.2151], 'dest', [-1 0]);
inst(1).basket_zone = circle(inst(1).dest, r_table_zone, 'n', n_table_zone);
inst(1).basket_traj = circle(inst(1).dest, r_table_traj, 'n', n_table_traj);
inst(2).shape = 'cylinder';
inst(2).pose = [-3.35; -5.92; 0.2151];
inst(2).dest = [-7 -3]; %dog
inst(2).basket_zone = circle(inst(2).dest, r_table_zone, 'n', n_table_zone);
inst(2).basket_traj = circle(inst(2).dest, r_table_traj, 'n', n_table_traj);
inst(3).shape = 'cylinder';
inst(3).pose = [-3.2; -5.725; 0.2351];
inst(3).dest = [7 7]; %pumpkin
inst(3).basket_zone = circle(inst(3).dest, r_table_zone, 'n', n_table_zone);
inst(3).basket_traj = circle(inst(3).dest, r_table_traj, 'n', n_table_traj);
inst(4).shape = 'box';
inst(4).pose = [-3; -5.65; 0.2451];
inst(4).dest = [-7 7]; %trike
inst(4).basket_zone = circle(inst(4).dest, r_table_zone, 'n', n_table_zone);
inst(4).basket_traj = circle(inst(4).dest, r_table_traj, 'n', n_table_traj);
inst(5).shape = 'box';
inst(5).pose = [-2.75; -5.8; 0.2151];
inst(5).dest = [7 3]; %plant
inst(5).basket_zone = circle(inst(5).dest, r_table_zone, 'n', n_table_zone);
inst(5).basket_traj = circle(inst(5).dest, r_table_traj, 'n', n_table_traj);


box_nb = 1;
goal = table1;


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

[res, homeGripperPosition] = ...
    vrep.simxGetObjectPosition(id, h.ptip,...
    h.armRef,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);

%%%%%%%%%%%%
fsm = 'exploration';
explorationComplete = false;
%%%%%%%%%%%%

[res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
[res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);

initial_rotation = 1;
traj_timer = tic;
elapsed_time = 0; % elapsed time since the begining  of a trajectory
next_point_delay = 1; % time interval before selecting the next pursuit point
new_traject = 1;

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
    
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
        vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
        vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    if strcmp(fsm, 'exploration')
        
        if plotData,
            % Read data from the Hokuyo sensor:
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
            
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
                if isempty(traj)
                    fsm = 'go to table/basket';
                    explorationComplete = true;
                end
                s = size(traj);
                if s(1) > 1
                    traj = smooth_traj(traj);
                    s = size(traj);
                end
                index = 1;
                
                new_traject = 0;
                traj_timer = tic;
                prev_t = 0;
                prev_e = 0.4;
            end
            
            if not(explorationComplete)
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
                i_star = floor((x_star+d)/cell_size)+1;
                j_star = floor((y_star+d)/cell_size)+1;
                if index == s(1) || map(i_star,j_star) == 1
                    new_traject = 1;
                end
                
                t = toc(traj_timer);
                e = sqrt((x_star-x)^2+(y_star-y)^2)-d_star;
                if e > 0.01
                    v_star = 20*e + 30*(abs(t-prev_t)*abs(e-prev_e)/2);
                    alpha = angdiff(theta_star, theta);
                    gamma = -theta+atan2((y_star - y),(x_star - x));
                    forwBackVel = v_star*sin(gamma);
                    leftRightVel = v_star*cos(gamma);
                    rotVel = alpha*(abs(forwBackVel)+abs(leftRightVel))/2;
                    
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
        end
        
    elseif strcmp(fsm, 'go to table/basket')
        
        if objectPickedUp
            circle_zone = inst(box_nb).basket_zone;
        else
            circle_zone = table1_zone;
        end
        
        if startGoing
            
            zone_index = find_closest_point(circle_zone, youbotPos(1), youbotPos(2));
            
            destIsObstacle = true;
            while destIsObstacle
                destIsObstacle = false;
                try
                    dest = [circle_zone(1,zone_index) circle_zone(2,zone_index)];
                    traj = calc_traj(map, youbotPos, dest, cell_size, d);
                catch
                    destIsObstacle = true;
                    if zone_index == n_table_zone
                        zone_index = 1;
                    else
                        zone_index = zone_index+1;
                    end
                end
            end
            
            s = size(traj);
            traj = [traj; dest];
            if s(1) > 1
                traj = smooth_traj(traj);
                s = size(traj);
            end
            index = 1;
            traj_timer = tic;
            prev_t = 0;
            prev_e = 0.4;
            startGoing = false;
        end
        
        x = youbotPos(1);
        x_star = traj(index,1);
        y = youbotPos(2);
        y_star = traj(index,2);
        theta = youbotEuler(3);
        theta_star = atan2((y_star - y),(x_star - x))+pi/2;
        
        t = toc(traj_timer);
        e = sqrt((x_star-x)^2+(y_star-y)^2)-d_star;
        if e > 0.01
            v_star = 20*e + 30*(abs(t-prev_t)*abs(e-prev_e)/2);
            alpha = angdiff(theta_star, theta);
            gamma = -theta+atan2((y_star - y),(x_star - x));
            forwBackVel = v_star*sin(gamma);
            leftRightVel = v_star*cos(gamma);
            rotVel = alpha*(abs(forwBackVel)+abs(leftRightVel))/2;
            
        else
            index = index + 1;
            if index > s(1)
                fsm = 'get close table/basket';
                startGetClose = true;
                forwBackVel = 0;
                leftRightVel = 0;
                rotVel = 0;
            end
        end
        
    elseif strcmp(fsm, 'get close table/basket')
        
        if startGetClose
            theta_star = table_tg_angle(goal, youbotPos(1:2));
            startGetClose = false;
            rotation_performed = false;
        end
        
        if not(rotation_performed)
            angdif = angdiff(theta_star, youbotEuler(3));
            rotVel = 10*angdif;
            if abs(angdif) < 1/180*pi,
                rotVel = 0;
                rotation_performed = true;
            end
        else
            e = sqrt((goal(1)-youbotPos(1))^2+(goal(2)-youbotPos(2))^2);
            leftRightVel = 10*e;
            
            if e <= 0.25+d_table/2
                leftRightVel = 0;
                
                if objectPickedUp
                    fsm = 'throw';
                else
                    fsm = 'round';
                    startRound = true;
                end
            end
        end
        
    elseif strcmp(fsm, 'round') % the youbot turn around a table
        
        [res, armPos] = vrep.simxGetObjectPosition(id, h.armRef, -1,...
            vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        
        x = youbotPos(1);
        y = youbotPos(2);
        theta = youbotEuler(3);
        
        if startRound
            circle_traj = table1_traj;
            
            index = find_closest_point(circle_traj, x, y) + 4;
            startRound = false;
            traj_timer = tic;
            prev_e = 0.1;
            prev_t = 0;
            v_supp = 0;
            
            box_pose = inst(box_nb).pose;
            x_box = box_pose(1);
            y_box = box_pose(2);
            
            index_dest = find_closest_point(circle_traj, x_box, y_box);
            x_dest = circle_traj(1,index_dest);
            y_dest = circle_traj(2,index_dest);
            
            if (index_dest > index && index_dest - index < n_table_traj/2)...
                    || (index_dest < index && index - index_dest > n_table_traj/2)
                
                direction = -1;
            else
                direction = 1;
            end
            
            dist1_arm_dest = 100;
        end
        
        x_star = circle_traj(1,index);
        y_star = circle_traj(2,index);
        if direction == 1
            theta_star = atan2((y_star - y),(x_star - x))-pi/2;
        else
            theta_star = atan2((y_star - y),(x_star - x))+pi/2;
        end
        
        t = toc(traj_timer);
        e = sqrt((x_star-x)^2+(y_star-y)^2)-0.1;
        if e > 0.01
            if v_supp < 0.8
                v_supp = (abs(t-prev_t)*abs(e-prev_e)/2);
            end
            v_star = 25*e + 10*v_supp + 2;
            alpha = 15*angdiff(theta_star, theta);
            %gamma = -theta+atan2((y_star - y),(x_star - x));
            forwBackVel = direction*v_star;%*sin(gamma);
            %leftRightVel = v_star*cos(gamma);
            rotVel = alpha;
            
        else
            if direction == 1
                if index == 1
                    index = length(circle_traj(1,:));
                else
                    index = index -1;
                end
            else
                if index == length(circle_traj(1,:))
                    index = 1;
                else
                    index = index +1;
                end
            end
            
            dist2_arm_dest = arc_dist(circle_traj, r_table_traj, [armPos(1); armPos(2)],[x_dest; y_dest]);
            DELTA_dist_arm_dest = dist2_arm_dest - dist1_arm_dest;
            dist1_arm_dest = dist2_arm_dest;
            
            if DELTA_dist_arm_dest > 0.01 %&& dist2_arm_dest < 0.1
                fsm = 'grab';
                forwBackVel = 0;
                leftRightVel = 0;
                rotVel = 0;
            end
        end
        
    elseif strcmp(fsm, 'grab')
        
        T = se2(youbotPos(1), youbotPos(2), youbotEuler(3));
        p = inst(box_nb).pose; %pose yellow box
        p(3) = p(3);
        p(1:2) = homtrans(inv(T), p(1:2));
        
        vrep.simxSetIntegerSignal(id, 'km_mode', 1, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        vrep.simxSetObjectPosition(id, h.ptarget, h.ref, p,...
            vrep.simx_opmode_oneshot_wait);
        
        gripTargDist = 1;
        while gripTargDist > 0.0005
            [res, tipPos] = vrep.simxGetObjectPosition(id, h.ptip, h.ref,...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            gripTargDist = sqrt((p(1)-tipPos(1))^2+(p(2)-tipPos(2))^2);
        end
        
        p(3) = p(3)-youbotPos(3)-0.015;
        vrep.simxSetObjectPosition(id, h.ptarget, h.ref, p,...
            vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        gripTargDist = 1;
        while gripTargDist > 0.0005
            [res, tipPos] = vrep.simxGetObjectPosition(id, h.ptip, h.ref,...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            gripTargDist = sqrt((p(1)-tipPos(1))^2+(p(2)-tipPos(2))^2);
        end
        pause(1);
        
        [res, a] = vrep.simxGetJointPosition(id, h.armJoints(4), vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(4),...
            a-7.5*pi/180,...
            vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        pause(1);
        
        vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        pause(1);
        
        vrep.simxSetIntegerSignal(id, 'km_mode', 1, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        p =[0; -0.1; youbotPos(3)+0.06];
        vrep.simxSetObjectPosition(id, h.ptarget, h.ref,...
            p,...
            vrep.simx_opmode_oneshot_wait);
        gripTargDist = 1;
        while gripTargDist > 0.0005
            [res, tipPos] = vrep.simxGetObjectPosition(id, h.ptip, h.ref,...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            gripTargDist = sqrt((p(1)-tipPos(1))^2+(p(2)-tipPos(2))^2);
        end
        
        pause(1);
        
        vrchk(vrep, res, true);
        vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        transportJoints = [0,30.91*pi/180,52.42*pi/180,0*pi/180,0];
        for i = 1:5
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                transportJoints(i),...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
        end
        pause(1);
        
        %         p =[0; -0.12; youbotPos(3)+0.02];
        %         vrep.simxSetObjectPosition(id, h.ptarget, h.ref,...
        %             p,...
        %             vrep.simx_opmode_oneshot_wait);
        %         gripTargDist = 1;
        %         while gripTargDist > 0.0005
        %             [res, tipPos] = vrep.simxGetObjectPosition(id, h.ptip, h.ref,...
        %                 vrep.simx_opmode_oneshot_wait);
        %             vrchk(vrep, res, true);
        %             gripTargDist = sqrt((p(1)-tipPos(1))^2+(p(2)-tipPos(2))^2);
        %         end
        %
        %         pause(1);
        
        objectPickedUp = true;
        goal = inst(box_nb).dest;
        startGoing = true;
        fsm = 'go to table/basket';
        
    elseif strcmp(fsm, 'throw')
        vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        throwJoints = [90*pi/180, 19.6*pi/180, 113*pi/180, -41*pi/180, 0*pi/180];
        for i = 1:5,
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                throwJoints(i),...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
        end
        pause(2);
        vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        pause(1);
        
        for i = 1:5,
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                startingJoints(i),...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
        end
        pause(1);
        
        box_nb = box_nb+1;
        if box_nb == 6
            fsm = 'finished';
        else
            goal = table1;
            fsm = 'go to table/basket';
            startGoing = true;
            objectPickedUp = false;
        end
        
        
    elseif strcmp(fsm, 'drive')
        forwBackVel = 10;
        
    elseif strcmp(fsm, 'finished')
        pause(1);
        break;
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
end% main function

