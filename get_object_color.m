function color = get_object_color(picture)
% Returns the color of the object on the picture, considering that the 
% point (75,256) of the image belong to the object
%
% ARGUMENTS 
%   - picture :  a picture token by the youbot in a specific way
%
% OUTPUT
%   - color :    a string representing the color of the object on the
%                picture

rgb = [picture(75,256,1), picture(75,256,2), picture(75,256,3)];

% The principle her is to evaluate the dominant composant of the color to
% determine of wich "color category" it belongs
value = max(rgb);
c = [0 0 0];
for i=1:3
    if value-10 < rgb(i) && rgb(i) < value+10
        c(i) = 1;
    end
end

if isequal(c, [0 1 0])
    color = 'green';
elseif  isequal(c, [0 0 1])
    color = 'blue';
elseif isequal(c, [1 0 0])
    color = 'red';
elseif isequal(c, [1 1 0])
    color = 'yellow';
elseif isequal(c, [1 0 1])
    color = 'purple';
elseif isequal(c, [1 1 1])
    if value > 128
        color = 'white';
    else 
        color = 'black';
    end
else
    color = 'undefined';
end

end