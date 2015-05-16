function updated_map = ptsToCellmap(X,Y,Xc,Yc,map,cell_size,pose,d,opt)

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
    
    if opt
        updated_map = ptsToCellmapMex(X,Y,double(Xc),double(Yc),map,cell_size,double(pose),d);
    else
		theta = pose(3);
        x_ref = pose(1);
        y_ref = pose(2);
        s = size(map);
        
        %map(floor((x_ref+d)/cell_size)+1, floor((y_ref+d)/cell_size)+1) = 0;

        for i = 1:length(X)

            m = floor((cos(theta)*X(i)-sin(theta)*Y(i)+x_ref+d)/cell_size);
            n = floor((sin(theta)*X(i)+cos(theta)*Y(i)+y_ref+d)/cell_size);
            if(m < 0)
                m = 0;
            end
            if(n < 0)
                n = 0;
            end
            if(m+1 <= s(1) && n+1 <= s(2) && map(m+1,n+1) ~= 1) 
                map(m+1,n+1) = 0;
            end
        end

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