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
recording_name = '../bagfiles/V2_lessprim_highway_6obs_5.bag';
recording = rosbag(recording_name);

topic_agent_pos = select(recording,'Topic',{'/jackal0/global_pos'});                           
msg_agent_pos = readMessages(topic_agent_pos,'DataFormat','struct'); 

topic_obs_pos = select(recording,'Topic',{'/obstacle/global_pos'});                           
msg_obs_pos = readMessages(topic_obs_pos,'DataFormat','struct'); 

topic_x_vel = select(recording,'Topic',{'/obstacle/x_vel'});                           
msg_x_vel = readMessages(topic_x_vel,'DataFormat','struct'); 

topic_y_vel = select(recording,'Topic',{'/obstacle/y_vel'});                           
msg_y_vel = readMessages(topic_y_vel,'DataFormat','struct'); 

topic_part_info = select(recording,'Topic',{'/particle_info_time'});
msg_part_info = readMessages(topic_part_info,'DataFormat','struct'); 

topic_pot_info = select(recording,'Topic',{'/pot_particle_info_time'});
global msg_pot_info
msg_pot_info = readMessages(topic_pot_info,'DataFormat','struct'); 

figure(1);
global endtime
endtime = size(msg_part_info{1}.Pt,2);
global time
global weight_points
global weight_parents
time = 0;
dt = 0.5;
while(time<endtime)
     clf(1);
     x_traj = zeros(size(msg_part_info{1}.Pt(time+1).Parray,2),time+1);
     y_traj = zeros(size(msg_part_info{1}.Pt(time+1).Parray,2),time+1);
     hold on 
     %plot(msg_agent_pos{1}.Position.Y,msg_agent_pos{1}.Position.X,".","MarkerSize",36,"Color","blue"); %reverse axis to be similar to gazebo
     plot(rail_pos_y,rail_x,"-",rail_neg_y,rail_x,"-","Color","black");

     %show obstacles
     for i=1:size(msg_obs_pos{1}.Poses,2)
        viscircles([msg_obs_pos{1}.Poses(i).Position.Y,msg_obs_pos{1}.Poses(i).Position.X+(msg_x_vel{1}.Data(i)*time*dt)],buffer,"Color","magenta");
        plot(msg_obs_pos{1}.Poses(i).Position.Y,msg_obs_pos{1}.Poses(i).Position.X+(msg_x_vel{1}.Data(i)*time*dt),".","MarkerSize",10,"Color","magenta");
     end
     for i=1:size(msg_part_info{1}.Pt(time+1).Parray,2)
        for j=1:time+1
            x_traj(i,j) = msg_part_info{1}.Pt(time+1).Parray(i).Pos(j).X;
            y_traj(i,j) = msg_part_info{1}.Pt(time+1).Parray(i).Pos(j).Y;
        end
        en = msg_part_info{1}.Pt(time+1).Parray(i).Energy;
        col = msg_part_info{1}.Pt(time+1).Parray(i).Colnum;
        pnum = msg_part_info{1}.Pt(time+1).Parray(i).Num;
        pid = msg_part_info{1}.Pt(time+1).Parray(i).ParticleId;
        
        plot(y_traj(i,time+1),x_traj(i,time+1),".","MarkerSize",10,"Color","blue",'Tag',sprintf("%d %f %d %d",pid, en, col, pnum));
        H(i) = plot(y_traj(i,:),x_traj(i,:),"-","MarkerSize",5,"Color","blue",'Tag',sprintf("%d %f %d %d",pid, en, col, pnum));

     end
     i = msg_part_info{1}.Pt(time+1).BestParticle+1;
     for j=1:time+1
         x_traj(i,j) = msg_part_info{1}.Pt(time+1).Parray(i).Pos(j).X;
         y_traj(i,j) = msg_part_info{1}.Pt(time+1).Parray(i).Pos(j).Y;
     end
     en = msg_part_info{1}.Pt(time+1).Parray(i).Energy;
     col = msg_part_info{1}.Pt(time+1).Parray(i).Colnum;
     pnum = msg_part_info{1}.Pt(time+1).Parray(i).Num;
     pid = msg_part_info{1}.Pt(time+1).Parray(i).ParticleId;
     plot(y_traj(i,time+1),x_traj(i,time+1),".","MarkerSize",10,"Color","cyan",'Tag',sprintf("%d %f %d %d",pid, en, col, pnum));
     H(i) = plot(y_traj(i,:),x_traj(i,:),"-","MarkerSize",5,"Color","cyan",'Tag',sprintf("%d %f %d %d",pid, en, col, pnum));
     
      if(time+1 < endtime)
         for i=1:size(msg_pot_info{1}.Potpt(time+1).Potparray,2)
              weight = msg_pot_info{1}.Potpt(time+1).Potparray(i).Weight;
              collision = msg_pot_info{1}.Potpt(time+1).Potparray(i).Collision;
              numpart = msg_pot_info{1}.Potpt(time+1).Potparray(i).NumParticles;
              %sprintf("%f %d %d",weight, collision, numpart)
              weight_points(i) = plot(msg_pot_info{1}.Potpt(time+1).Potparray(i).Pos.Y,msg_pot_info{1}.Potpt(time+1).Potparray(i).Pos.X,".","MarkerSize",20,"Color","green",'Tag',sprintf("%f %d %d",weight, collision, numpart));
              weight_parents(i) = msg_pot_info{1}.Potpt(time+1).Potparray(i).ParentId;
              set(weight_points(i),'Visible','off');
         end
      end
     set(H, 'ButtonDownFcn', {@LineSelected, H})
     set(gca, 'YDir','reverse') 
     axis equal
     ylim([xmin xmax])
     xlim([ymin ymax])
     
     pause;
     time = time+1;
     hold off;
     
end
% 
% 
% 
% 
% % function txt = myupdatefcn(~,event)
% % pos = get(event,'Position');
% % dts = str2num(get(event.Target,'Tag'));
% % %set(H(dts), 'LineWidth', 2.5);
% % %set(H(H ~= dts), 'LineWidth', 0.5);
% % txt = {dts,
% %        ['X: ',num2str(pos(1))],
% %      ['Y: ',num2str(pos(2))]};
% % end
% 
function LineSelected(ObjectH, EventData, H) 
    set(ObjectH, 'LineWidth', 2.5);
    set(H(H ~= ObjectH), 'LineWidth', 0.5);
    str_sp = split((get(ObjectH,'Tag')));
    id = str2double(str_sp(1));
    en = str2double(str_sp(2));
    cnum = str2double(str_sp(3));
    num = str2double(str_sp(4));
    global time
    global weight_points
    global weight_parents
    for j=1:size(weight_points,2)
        if(weight_parents(j)==id)
            if isgraphics(weight_points(j))
                set(weight_points(j),'Visible','on');
                set(weight_points(j), 'ButtonDownFcn', {@wSelected, id})
            end
        else
            if isgraphics(weight_points(j))
                set(weight_points(j),'Visible','off');
            end
        end
    end
    subtitle("")
    title(sprintf("Timestep: %d, particle: %d, ColNum: %d, Num of Particles: %d, Energy: %f",time, id, cnum, num, en))
end

function wSelected(ObjectH, ~, id)
    global weight_points
    global weight_parents
    for j=1:size(weight_points,2)
        if(weight_parents(j)==id)
            set(weight_points(j),'Color', "green");
        end
    end
    set(ObjectH, 'Color', "red");
    str_sp = split((get(ObjectH,'Tag')));
    weight = str2double(str_sp(1));
    col = str2double(str_sp(2));
    numPart = str2double(str_sp(3));
    %subtitle("WEIGHT")
    subtitle(sprintf("Weight: %f, Collision: %d, Num of Particles: %d",weight, col, numPart))
end
% function plotButtonPushed(~)
%     global time
%     time = time+1;
% end