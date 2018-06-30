filename = 'NewMap';

map = csvread([filename,'.csv']);
SectNum = length(map(:,1))/3;
figure
plot(0,0,'k+','MarkerSize',8);
hold on;
pause(2);

xlim([-6,6]);
ylim([-1.5,10]);
for i=1:SectNum
%     tag=find(map(3*i,:)==100);
%     PointNum = tag-1;
    
    tag=find(map(3*i,:)==0);
    PointNum = tag(1)-1;
    %     PointNum = length(map(3*i,:));
    fprintf('Plot points in section %d \n',i);
    for j=1:PointNum
        switch map(3*i,j)
            case -1 % temp
                marker = 'k.';
            case 1 % room corner
                marker = 'ro';
            case 2 % exit corner
                marker = 'go';
        end
        plot(-map(3*i-1,j),map(3*i-2,j), marker,'MarkerSize',6); % plot(-y,x), the same coord of pico
        fprintf('Plot num %d point\n',j);
        pause(0.2);
    end
    
    for j=1:PointNum-1
        line([-map(3*i-1,j),-map(3*i-1,j+1)],[map(3*i-2,j),map(3*i-2,j+1)]);
        pause(0.2);
    end
end