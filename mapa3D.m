%% Código de generacion de mapa local en 3D
% Para visualizar una zona específica del mapa es necesario cargar el .bag
% correspondiente y lanzar el for que lo visualiza

%% Cargamos los elementos del mapa 3D que serán fijos para todo el código
% Cargamos los datos generados en el LOAM sobre la localizacion
LOAM = importdata('Odom.txt');% Odometria del LOAM
mapa = pcread('map.pcd');% Nube de puntos casadas mediante el LOAM
mapa = pcdenoise(mapa); % Eliminamos ruido en la nube de puntos
map3D = occupancyMap3D(5);
puntosMap = mapa.Location;

% Quitamos los elementos para mejorar la visualización  
for j=1:length(puntosMap)
    if puntosMap(j,3) > 1.3 % Quitamos el techo
        puntosMap(j,:)= 0;
    end
    % Quitamos elementos del laser lejanos o que se ven a través de
    % cristales (EJE X)
    if puntosMap(j,1) <=-15 || puntosMap(j,1) >= 35 
        puntosMap(j,:)= 0;
    end
    % Quitamos elementos del laser lejanos o que se ven a través de
    % cristales (EJE Y)
    if puntosMap(j,2) <=-10 || puntosMap(j,2) >= 8 
        puntosMap(j,:)= 0;
    end
    
end

% Mostramos la nube de puntos por pantalla
figure(1)
pcshow(puntosMap);
title('Nube de puntos generada con LOAM');
axis equal
%% Cargamos los .bag con los datos grabados
% Bag 1

% Leemos el .bag
bag1 = rosbag('pasillo-009.bag');

% Leemos Odom
selPose1 = select(bag1,'Topic','/robot/robotnik_base_control/odom');
msgPose1 = readMessages(selPose1,'DataFormat','struct');
%Leemos point cloud:
selOuster1 = select(bag1,'Topic','/os_cloud_node/points');
msgOuster1 = readMessages(selOuster1);%,'DataFormat','struct');

% Nos quedamos con el mismo número de msg para láser Y Odom
[~,idx]=min([length(msgOuster1) length(msgPose1)]);

if idx==1 %Láser tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgOuster1,msgPose1);
    msgPose1=msgPose1(indexes);
    timeStamps1 = selOuster1.MessageList.Time;
elseif idx==2 %Odometría tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgPose1,msgOuster1);
    msgOuster1=msgOuster1(indexes);
    timeStamps1 = selPose1.MessageList.Time;
end

%% Generamos el mapa de ocupación 3D

for i=1:length(timeStamps1) 
    xyz = readXYZ(msgOuster1{i}); %% Corregir si hace falta
    %Quitamos el techo del octomap
    for j=1:length(xyz)
        if xyz(j,3) > 1.3
            xyz(j,:)= 0;
        end
    end
    % Introducimos la nube de puntos en el mapa 3D
    insertPointCloud(map3D,[LOAM(i,1) LOAM(i,2) LOAM(i,3) LOAM(i,4) LOAM(i,5) LOAM(i,6) LOAM(i,7)],xyz,6);
    
    % Mostramos los elementos por la pantalla
    figure(2)
    show(map3D)
    title('Octomap local');
    axis equal
    map3D = occupancyMap3D(5);
    disp(i);
    refreshdata
    drawnow limitrate
end


%% Bag 2
bag2 = rosbag('pasillo-007.bag');

selPose2 = select(bag2,'Topic','/robot/robotnik_base_control/odom');
msgPose2 = readMessages(selPose2);
%Leemos point cloud:
selOuster2 = select(bag2,'Topic','/os_cloud_node/points');
msgOuster2 = readMessages(selOuster2);%,'DataFormat','struct');

% Nos quedamos con el mismo número de msg para láser, odom
[~,idx]=min([length(msgOuster2) length(msgPose2)]);

if idx==1 %Láser tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgOuster2,msgPose2);
    msgPose22=msgPose2(indexes);
    timeStamps22 = selOuster2.MessageList.Time;
elseif idx==2 %Odometría tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgPose2,msgOuster2);
    msgOuster22=msgOuster2(indexes);
    timeStamps22 = selPose2.MessageList.Time;
end

%% Generamos el mapa de ocupación 3D

for i=1:length(timeStamps2)
    xyz = readXYZ(msgOuster2{i}); %% Corregir si hace falta
    %Quitamos el techo del octomap
    for j=1:length(xyz)
        if xyz(j,3) > 1.3
            xyz(j,:)= 0;
        end
    end
    % Introducimos la nube de puntos en el mapa 3D
    insertPointCloud(map3D,[LOAM(i,1) LOAM(i,2) LOAM(i,3) LOAM(i,4) LOAM(i,5) LOAM(i,6) LOAM(i,7)],xyz,6);
    
    % Mostramos los elementos por la pantalla
    figure(2)
    show(map3D)
    title('Octomap local');
    axis equal
    map3D = occupancyMap3D(5);
    disp(i);
    refreshdata
    drawnow limitrate
end




%% Bag 3
bag3 = rosbag('pasillo-002.bag');

selPose3 = select(bag3,'Topic','/robot/robotnik_base_control/odom');
msgPose3 = readMessages(selPose3,'DataFormat','struct');
%Leemos point cloud:
selOuster3 = select(bag3,'Topic','/os_cloud_node/points');
msgOuster3 = readMessages(selOuster3);%,'DataFormat','struct');

% Nos quedamos con el mismo número de msg para láser, odom 
[~,idx]=min([length(msgOuster3) length(msgPose3)]);

if idx==1 %Láser tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgOuster3,msgPose3);
    msgPose3=msgPose3(indexes);
    timeStamps3 = selOuster3.MessageList.Time;
elseif idx==2 %Odometría tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgPose3,msgOuster3);
    msgOuster3=msgOuster3(indexes);
    timeStamps3 = selPose3.MessageList.Time;
end

%% Generamos el mapa de ocupación 3D

for i=1:length(timeStamps3)
    xyz = readXYZ(msgOuster3{i}); %% Corregir si hace falta
    %Quitamos el techo del octomap
    for j=1:length(xyz)
        if xyz(j,3) > 1.3
            xyz(j,:)= 0;
        end
    end
    % Introducimos la nube de puntos en el mapa 3D
    insertPointCloud(map3D,[LOAM(i,1) LOAM(i,2) LOAM(i,3) LOAM(i,4) LOAM(i,5) LOAM(i,6) LOAM(i,7)],xyz,6);
    
    % Mostramos los elementos por la pantalla
    figure(2)
    show(map3D)
    title('Octomap local');
    axis equal
    map3D = occupancyMap3D(5);
    disp(i);
    refreshdata
    drawnow limitrate
end



%% Bag 4
bag4 = rosbag('pasillo-005.bag');

selPose4 = select(bag4,'Topic','/robot/robotnik_base_control/odom');
msgPose4 = readMessages(selPose4,'DataFormat','struct');
%Leemos point cloud:
selOuster4 = select(bag4,'Topic','/os_cloud_node/points');
msgOuster4 = readMessages(selOuster4);%,'DataFormat','struct');

% Nos quedamos con el mismo número de msg para láser, odom y TfOdom
[~,idx]=min([length(msgOuster4) length(msgPose4)]);

if idx==1 %Láser tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgOuster4,msgPose4);
    msgPose4=msgPose4(indexes);
    timeStamps4 = selOuster4.MessageList.Time;
elseif idx==2 %Odometría tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgPose4,msgOuster4);
    msgOuster4=msgOuster4(indexes);
    timeStamps4 = selPose4.MessageList.Time;
end


%% Generamos el mapa de ocupación 3D

for i=1:length(timeStamps4)
    xyz = readXYZ(msgOuster4{i}); %% Corregir si hace falta
    %Quitamos el techo del octomap
    for j=1:length(xyz)
        if xyz(j,3) > 1.3
            xyz(j,:)= 0;
        end
    end
    % Introducimos la nube de puntos en el mapa 3D
    insertPointCloud(map3D,[LOAM(i,1) LOAM(i,2) LOAM(i,3) LOAM(i,4) LOAM(i,5) LOAM(i,6) LOAM(i,7)],xyz,6);
    
    % Mostramos los elementos por la pantalla
    figure(2)
    show(map3D)
    title('Octomap local');
    axis equal
    map3D = occupancyMap3D(5);
    disp(i);
    refreshdata
    drawnow limitrate
end



%% Bag 5
bag5 = rosbag('pasillo-006.bag');

selPose5 = select(bag5,'Topic','/robot/robotnik_base_control/odom');
msgPose5 = readMessages(selPose5,'DataFormat','struct');
%Leemos point cloud:
selOuster5 = select(bag5,'Topic','/os_cloud_node/points');
msgOuster5 = readMessages(selOuster5);%,'DataFormat','struct');

% Nos quedamos con el mismo número de msg para láser, odom y TfOdom
[~,idx]=min([length(msgOuster5) length(msgPose5)]);

if idx==1 %Láser tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgOuster5,msgPose5);
    msgPose5=msgPose5(indexes);
    timeStamps5 = selOuster5.MessageList.Time;
elseif idx==2 %Odometría tiene menos mensajes
    [indexes, fromTimes, toTimes, diffs] = matchTimestamps(msgPose5,msgOuster5);
    msgOuster5 =msgOuster5(indexes);
    timeStamps5 = selPose5.MessageList.Time;
end

%% Generamos el mapa de ocupación 3D

for i=1:length(timeStamps5)
    xyz = readXYZ(msgOuster5{i});
    %Quitamos el techo del octomap
    for j=1:length(xyz)
        if xyz(j,3) > 1.3
            xyz(j,:)= 0;
        end
    end
    %insertPointCloud(map3D,[msgPose{i}.Pose.Pose.Position.X msgPose{i}.Pose.Pose.Position.Y msgPose{i}.Pose.Pose.Position.Z msgPose{i}.Pose.Pose.Orientation.W msgPose{i}.Pose.Pose.Orientation.X msgPose{i}.Pose.Pose.Orientation.Y msgPose{i}.Pose.Pose.Orientation.Z],xyz,6);
    insertPointCloud(map3D,[LOAM(i,1) LOAM(i,2) LOAM(i,3) LOAM(i,4) LOAM(i,5) LOAM(i,6) LOAM(i,7)],xyz,6);
    
    % Mostramos los elementos por la pantalla
    figure(1)
    show(map3D)
    title('Octomap local');
    axis equal
    map3D = occupancyMap3D(5);
    disp(i);
    refreshdata
    drawnow limitrate
end


