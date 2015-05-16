elseif strcmp(fsm, 'grab')
        
        T = se2(youbotPos(1), youbotPos(2), youbotEuler(3));
        %p = [-3.2; -5.725; 0.2351]; %pose red cyl
        p = [-3; -5.65; 0.2451]; %pose yellow box
        %p = [-1.025;-5.75;0.185]
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
        
        p(3) = p(3)-youbotPos(3);
        vrep.simxSetObjectPosition(id, h.ptarget, h.ref, p,...
            vrep.simx_opmode_oneshot_wait);
        
        vrchk(vrep, res, true);
        pause(3);
        vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        pause(1)
        p =[-0.08; -0.1; youbotPos(3)+0.06];
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
        
        pause(2);
        
        vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        
        p =[-0.09; -0.11; youbotPos(3)+0.2];
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
        
        %%%%%%%%%%%
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        T = se2(youbotPos(1), youbotPos(2), youbotEuler(3));
        p = [-3.2; -5.725; 0.2351];
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
        
        p(3) = p(3)-youbotPos(3);
        vrep.simxSetObjectPosition(id, h.ptarget, h.ref, p,...
            vrep.simx_opmode_oneshot_wait);
        
        vrchk(vrep, res, true);
        pause(3);
        vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        pause(1)
        p =[0; -0.11; youbotPos(3)+0.06];
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
        
        pause(2);
        
        vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        
        %%%%%%%%%%%
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
            vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        T = se2(youbotPos(1), youbotPos(2), youbotEuler(3));
        p = [-2.75; -5.8; 0.2151]; %red box
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
        
        p(3) = p(3)-youbotPos(3);
        vrep.simxSetObjectPosition(id, h.ptarget, h.ref, p,...
            vrep.simx_opmode_oneshot_wait);
        
        vrchk(vrep, res, true);
        pause(3);
        vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        pause(1)
        p =[0.08; -0.13; youbotPos(3)+0.06];
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
        
        pause(2);
        
        
        vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        
        fsm = 'drive';