function updated_map = ptsToCellmap(X,Y,Xc,Yc,map,cell_size,pose,d,opt)
% Updates a map based on the data gathered by Hokuyo sensors of a robot
%
% ARGUMENTS
%   - X :           a vector containing the x coordinates of the points in 
%                   the area swept by the sensors. These coordinates are in
%                   the reference frame of the robot.
%   - Y :           a vector containing the y coordinates of the points in 
%                   the area swept by the sensors. These coordinates are in
%                   the reference frame of the robot.
%   - Xc :          a vector containing the x coordinates of the points 
%                   where obstacles were detected by the sensors. These 
%                   coordinates are in the reference frame of the robot.
%   - Yc :          a vector containing the y coordinates of the points 
%                   where obstacles were detected by the sensors. These 
%                   coordinates are in the reference frame of the robot.
%   - map :         a NxN matrix representing the occupancy map in wich we
%                   want to calculate the trajectory. The reference frame
%                   is supposed to be at the center of the map. Unexplored
%                   points in the map are equal to -1, explored ones are
%                   equal to 0 and obstacles are equal to 1.
%   - cell_size :   size of the side of a cell in the occupancy map
%   - pose :        the position [x y theta] of the frame of the robot in
%                   the reference frame of the map.
%   - d :           distance between the reference frame and a frame in the
%                   corner of the map.
%   - opt :         Boolean value, true if you want to use the mex
%                   implementation, and false if you want to use the matlab
%                   one.
%
% OUTPUT
%   - updated_map : The map that has been updated.

% Check the type of the arguments
    if nargin < 9
        if nargin < 8
            if nargin < 7
                error('not enough input arguments');
            end
            d = 8;
        end
        opt = false;
    end
   
    msg = '';
	if ~isreal(X) || ~isvector(X) || ~isa(X, 'double')
		msg = [msg 'X must be a real 1D double array.' char(10)];
	end
	if ~isreal(Y) || ~isvector(Y) || ~isa(Y, 'double')
		msg = [msg 'Y must be a real 1D double array.' char(10)];
	end
	if numel(X) ~= numel(Y)
		msg = [msg 'X and Y must be the same size.' char(10)];
	end
	if ~isreal(Xc) || ~isvector(Xc) 
		msg = [msg 'Xc must be a real 1D double array.' char(10)];
	end
	if ~isreal(Yc) || ~isvector(Yc) 
		msg = [msg 'Yc must be a real 1D double array.' char(10)];
	end
	if numel(Xc) ~= numel(Yc)
		msg = [msg 'Xc and Yc must be the same size.' char(10)];
	end
	if ~isreal(map) || ~ismatrix(map) || ~isa(map, 'double')
		msg = [msg 'map must be a real 2D double array.' char(10)];
    end
    if ~isreal(cell_size) || numel(cell_size) ~= 1 || ~isa(cell_size, 'double') 
		msg = [msg 'cell_size must be a real double scalar.' char(10)];
    end
    if ~isreal(pose) || ~isvector(pose) ||  numel(pose) ~= 3
        msg = [msg 'pose must be a real 1D double array with 3 elements.' char(10)];
    end
    if ~isreal(d) || numel(d) ~= 1 || ~isa(d, 'double') 
		msg = [msg 'd must be a real double scalar.' char(10)];
    end
              
    if ~strcmp(msg, '')
        error(msg);
    end
    
    if opt % performs the update using the mex function (faster)
        updated_map = ptsToCellmapMex(X,Y,double(Xc),double(Yc),map,cell_size,double(pose),d);
    else % performs the updtate using a matlab implementation (slower)
		theta = pose(3);
        x_ref = pose(1);
        y_ref = pose(2);
        s = size(map);

        % Adding explored points to the map
        for i = 1:length(X)

            m = floor((cos(theta)*X(i)-sin(theta)*Y(i)+x_ref+d)/cell_size);
            n = floor((sin(theta)*X(i)+cos(theta)*Y(i)+y_ref+d)/cell_size);
            if(m < 0)
                m = 0;
            end
            if(n < 0)
                n = 0;
            end
            % Do not mark as explored a point that has already been
            % identified as an obstacle
            if(m+1 <= s(1) && n+1 <= s(2) && map(m+1,n+1) ~= 1) 
                map(m+1,n+1) = 0;
            end
        end

        % Adding obstacles to the  map 
        for i = 1:length(Xc)

            m = floor((cos(theta)*Xc(i)-sin(theta)*Yc(i)+x_ref+d)/cell_size);
            n = floor((sin(theta)*Xc(i)+cos(theta)*Yc(i)+y_ref+d)/cell_size);
            
            if(m < 0)
                m = 0;
            end
            if(n < 0)
                n = 0;
            end
            if(m+1 <= s(1) && n+1 <= s(2))
                map(m+1,n+1) = 1;
            end
        end

        updated_map = map;
    end
end