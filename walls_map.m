function newMap = walls_map(map, tableList, d_table, cell_size, d)

newMap = map;
r = d_table/2;

for i = 1:length(tableList(:,1))
    x1 = floor((tableList(i,1)-r+d)/cell_size)+1;
    y1 = floor((tableList(i,2)-r+d)/cell_size)+1;
    x2 = floor((tableList(i,1)+r+d)/cell_size)+1;
    y2 = floor((tableList(i,2)+r+d)/cell_size)+1;
    
    newMap(x1:x2,y1:y2) = 0;
end    

