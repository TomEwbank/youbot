function color = get_object_color(shot)

rgb = [shot(1,256,1), shot(1,256,2), shot(1,256,3)]
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