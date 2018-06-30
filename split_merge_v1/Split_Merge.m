%% up corner 1
clear;
Wall=[0.0231776,-2.06257,3.63606,-2.07472,0.662913,-0.327168,0.649774,2.04582,0.0150264,2.07749];
Corner=[0.649774,2.04582];
Line=[-0.00336192,-1,-2.0625,-3.71154,-1,2.13326,-0.0498853,-1,2.07824];

WallNum=length(Wall)/2;
CornerNum=length(Corner)/2;
LineNum=length(Line)/3;

figure
plot(0,0,'k+','linewidth',2);
hold on;
for i=1:WallNum
    plot(Wall(2*i-1),Wall(2*i),'ro','linewidth',2);
end
for i=1:CornerNum
    plot(Corner(2*i-1),Corner(2*i),'b+','markers',12,'linewidth', 3);
end

x=linspace(-1,4,4);
for i=1:LineNum
    k=Line(3*i-2);
    b=Line(3*i);
    y=x*k+b;
    plot(x,y);
end
xlim([-1,5]);
ylim([-3,3]);
view([-90 90])

%% emc-sim
clear;

Wall=[0.00616846,-0.548931,1.11325,-0.549423,2.18817,-1.02598,2.18655,1.92831,1.11566,1.02433,0.00741602,1.02531];
% Corner=[];
Line=[-0.000444314,-1,-0.548928,-1819.6,-1,3980.57,-0.000889377,-1,1.02532];

WallNum=length(Wall)/2;
% CornerNum=length(Corner)/2;
LineNum=length(Line)/3;

figure
plot(0,0,'k+','linewidth',2);
hold on;
for i=1:WallNum
    plot(Wall(2*i-1),Wall(2*i),'ro','linewidth',2);
end
% for i=1:CornerNum
%     plot(Corner(2*i-1),Corner(2*i),'b+','markers',12,'linewidth', 3);
% end

x=linspace(-1,4,4);
for i=1:LineNum
    k=Line(3*i-2);
    b=Line(3*i);
    y=x*k+b;
    plot(x,y);
end
xlim([-1,5]);
ylim([-3,3]);
view([-90 90])

%% hospital
clear;

Wall=[0.00458393,-0.407923,1.20571,-0.42425,2.15689,-0.523801,2.22283,-0.400305,2.93445,-0.407958,2.44682,0.400297,0.00288162,0.398402];
Corner=[2.22283,-0.400305];
Line=[-0.013593,-1,-0.407861,0.14898,-1,-0.845135,-0.0107553,-1,-0.376397,0.000775278,-1,0.3984];

WallNum=length(Wall)/2;
CornerNum=length(Corner)/2;
LineNum=length(Line)/3;

figure
plot(0,0,'k+','linewidth',2);
hold on;
for i=1:WallNum
    plot(Wall(2*i-1),Wall(2*i),'ro','linewidth',2);
end
for i=1:CornerNum
    plot(Corner(2*i-1),Corner(2*i),'b+','markers',12,'linewidth', 3);
end

x=linspace(-1,4,4);
for i=1:LineNum
    k=Line(3*i-2);
    b=Line(3*i);
    y=x*k+b;
    plot(x,y);
end
xlim([-1,5]);
ylim([-3,3]);
view([-90 90])


%% room-middle-3
clear;

Wall=[0.0238721,-2.12438,0.731235,-2.12404,0.849016,-0.0493477,3.38113,0.553148,0.800989,0.575011,0.774966,2.13913,0.0154042,2.12973];
Corner=[0.731235,-2.12404,0.800989,0.575011,0.774966,2.13913];
Line=[2.51475,-1,-2.18441,17.6149,-1,-15.0047,-0.468424,-1,2.13695,-1.97906,-1,2.16022,0.0123727,-1,2.12954];

WallNum=length(Wall)/2;
CornerNum=length(Corner)/2;
LineNum=length(Line)/3;

figure
plot(0,0,'k+','linewidth',2);
hold on;
for i=1:WallNum
    plot(Wall(2*i-1),Wall(2*i),'ro','linewidth',2);
end
for i=1:CornerNum
    plot(Corner(2*i-1),Corner(2*i),'b+','markers',12,'linewidth', 3);
end

x=linspace(-1,4,4);
for i=1:LineNum
    k=Line(3*i-2);
    b=Line(3*i);
    y=x*k+b;
    plot(x,y);
end
xlim([-1,5]);
ylim([-3,3]);
view([-90 90])

%% plot line directly room-middle-2
figure
plot(0,0,'k+','MarkerSize',10);
xlim([-1,5]);
ylim([-3,3]);
hold on;
line([0.0238789,0.731612,0.84838],[-2.12498,-2.12514,-0.0493107]);
line([3.38047,0.800958,0.775099,0.0154015],[0.55304,0.574988,2.1395,2.12935]);
view([-90 90]);

%% plot line directly hospital
figure
plot(0,0,'k+','MarkerSize',10);
xlim([-1,5]);
ylim([-3,3]);
hold on;
line([0.0046388,1.20643],[-0.412806,-0.424506]);
line([2.15577,2.2232,2.93144],[-0.52353,-0.400372,-0.40754]);
line([2.44417,0.00289758],[0.399863,0.400609]);
view([-90 90]);

%% plot line directly hospital
figure
plot(-1,0,'k+','MarkerSize',10);
xlim([-2,3]);
ylim([-4,3]);
hold on;
line([-0.995545,0.206476,1.63332],[-1.23656,-3.3064,-2.49703]);
line([0.827612,-0.0577283,1.13757],[-1.66459,-0.104306,0.585027]);
line([2.14574,1.0531],[0.923761,2.8504]);
line([0.334296,-1.00106],[1.94028,1.18137]);
view([-90 90]);
