clc
clear
close all
xmin = -20;
xmax = -10;
ymin = -2;
ymax = 2;
rail = 0.8;
buffer = 0.8;
button_press = 0;
if(rail>0)
    rail_x = linspace(xmin, xmax,10);
    rail_pos_y = ones(10)*rail;
    rail_neg_y = ones(10)*rail*-1;
end
recording_name = '../bagfiles/lessprim_highway_6obs.bag';
recording = rosbag(recording_name);

topic_agent_pos = select(recording,'Topic',{'/jackal0/global_pos'});                           
msg_agent_pos = readMessages(topic_agent_pos,'DataFormat','struct'); 

topic_obs_pos = select(recording,'Topic',{'/obstacle/global_pos'});                           
msg_obs_pos = readMessages(topic_obs_pos,'DataFormat','struct'); 

topic_x_vel = select(recording,'Topic',{'/obstacle/x_vel'});                           
msg_x_vel = readMessages(topic_x_vel,'DataFormat','struct'); 

topic_y_vel = select(recording,'Topic',{'/obstacle/y_vel'});                           
msg_y_vel = readMessages(topic_y_vel,'DataFormat','struct'); 

topic_part_info = select(recording,'Topic',{'/particle_info'});                           
msg_part_info = readMessages(topic_part_info,'DataFormat','struct'); 

figure(1);
%fig2 = uifigure;
%fig2.Position = [100 100 100 80];
% btn = uibutton(fig2,'push',...
%                'Position',[0, 0, 100, 80],...
%                'ButtonPushedFcn', @(btn,event) plotButtonPushed(btn));
% ax = uiaxes(fig2);
endtime = size(msg_part_info{1}.Parray(1).Pos,2);
time = 0;
dt = 0.5;
while(time<endtime)
    clf(1);
    x_traj = zeros(size(msg_part_info{1}.Parray,2),time+1);
    y_traj = zeros(size(msg_part_info{1}.Parray,2),time+1);
    %show starting position and rails
    %viscirccles([msg_agent_pos{1}.Position.Y,msg_agent_pos{1}.Position.X],buffer,"Color","blue");
    hold on 
    plot(msg_agent_pos{1}.Position.Y,msg_agent_pos{1}.Position.X,".","MarkerSize",36,"Color","blue"); %reverse axis to be similar to gazebo
    plot(rail_pos_y,rail_x,"-",rail_neg_y,rail_x,"-","Color","black");

    %show obstacles
    for i=1:size(msg_obs_pos{1}.Poses,2)
        viscircles([msg_obs_pos{1}.Poses(i).Position.Y,msg_obs_pos{1}.Poses(i).Position.X+(msg_x_vel{1}.Data(i)*time*dt)],buffer,"Color","magenta");
        plot(msg_obs_pos{1}.Poses(i).Position.Y,msg_obs_pos{1}.Poses(i).Position.X+(msg_x_vel{1}.Data(i)*time*dt),".","MarkerSize",20,"Color","magenta");
    end
    
    %show each particle
    for i=1:size(msg_part_info{1}.Parray,2)
        for j=1:time+1
            x_traj(i,j) = msg_part_info{1}.Parray(i).Pos(j).X;
            y_traj(i,j) = msg_part_info{1}.Parray(i).Pos(j).Y;
        end
        plot(msg_part_info{1}.Parray(i).Pos(time+1).Y,msg_part_info{1}.Parray(i).Pos(time+1).X,".","MarkerSize",10,"Color","blue",'Tag',sprintf("%d",i));
        en = msg_part_info{1}.Parray(i).Energy;
        cnum = msg_part_info{1}.Parray(i).Colnum;
        num = msg_part_info{1}.Parray(i).Num;
        H(i) = plot(y_traj(i,:),x_traj(i,:),"-","MarkerSize",5,"Color","blue",'Tag',sprintf("%d %f %d %d",i, en, cnum, num));
    end
    set(H, 'ButtonDownFcn', {@LineSelected, H})
    set(gca, 'YDir','reverse') 
    axis equal
    ylim([xmin xmax])
    xlim([ymin ymax])
%     datacursormode on
%     dcm = datacursormode(gcf);
%     set(dcm,'UpdateFcn',@myupdatefcn)
    time = time+1;
    hold off;
    pause
    %pause(dt);
end
hold on
best_x = ones(1,endtime);
best_y = ones(1,endtime);
for i=1:endtime
    best_x(i) = msg_part_info{1}.Parray(msg_part_info{1}.BestParticle+1).Pos(i).X;
    best_y(i) = msg_part_info{1}.Parray(msg_part_info{1}.BestParticle+1).Pos(i).Y;
    %plot(msg_part_info{1}.Parray(msg_part_info{1}.BestParticle+1).Pos(i).Y,,"-","MarkerSize",20,"Color","cyan")
end
en = msg_part_info{1}.Parray(msg_part_info{1}.BestParticle+1).Energy;
cnum = msg_part_info{1}.Parray(msg_part_info{1}.BestParticle+1).Colnum;
num = msg_part_info{1}.Parray(msg_part_info{1}.BestParticle+1).Num;
H(msg_part_info{1}.BestParticle+1) = plot(best_y,best_x,"-","MarkerSize",20,"Color","cyan",'Tag',sprintf("%d %f %d %d",i, en, cnum, num));
set(H, 'ButtonDownFcn', {@LineSelected, H})


% function txt = myupdatefcn(~,event)
% pos = get(event,'Position');
% dts = str2num(get(event.Target,'Tag'));
% %set(H(dts), 'LineWidth', 2.5);
% %set(H(H ~= dts), 'LineWidth', 0.5);
% txt = {dts,
%        ['X: ',num2str(pos(1))],
%      ['Y: ',num2str(pos(2))]};
% end

function LineSelected(ObjectH, EventData, H)
    set(ObjectH, 'LineWidth', 2.5);
    set(H(H ~= ObjectH), 'LineWidth', 0.5);
    str_sp = split((get(ObjectH,'Tag')));
    i = str2double(str_sp(1));
    en = str2double(str_sp(2));
    cnum = str2double(str_sp(3));
    num = str2double(str_sp(4));
    title(sprintf("particle: %d, ColNum: %d, Num of Particles: %d, Energy: %f",i, cnum, num, en))
end
% function plotButtonPushed(~)
%     global time
%     time = time+1;
% end