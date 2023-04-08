function Data_EFL = DataExtract(Id_set,LC_Frame,tracks)
%% 输出三辆车的运动信息(x,y,vx,vy,ax,ay)
global M2
% 各路段到最迟换道点的距离
keySet1 = {1990,1991,1994,1995,1992,1996,2155,2156,2222,2223,2157,2158,2010,2001,1998,2002 2228 2003};
valueSet1 = [274.64 243.85 274.11 244.34 216.24 216.24 193.91 193.91 170.73 170.73 141.37 141.37 8.06 8.06 4.94 4.94 -17.66 -33.52];
M1 = containers.Map(keySet1,valueSet1);
% 各路段位于左车道还是右车道(0表示左车道，1表示右车道)
keySet2 = {1990,1991,1992,2155,2222,2157,2010,1998,2226,1999,1994,1995,1996,2156,2223,2158,2001,2002,2228,2003,2073};
valueSet2 = [0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1];
M2 = containers.Map(keySet2,valueSet2);
Start_Frame = zeros(1,size(Id_set,1));
End_Frame = zeros(1,size(Id_set,1));
for i=1:size(Id_set,1)
    %% 确定提取数据的长度（从XX-XX全局帧）
    % Ego车辆
    Top_ev = find(tracks(Id_set(i,1)+1).laneletId==1991);
    Top_EV = Top_ev(1)+tracks(Id_set(i,1)+1).initialFrame-1;
    End_EV = tracks(Id_set(i,1)+1).finalFrame;
    % Front 车辆
    LCF_FV = find(tracks(Id_set(i,2)+1).laneChange==1);
    if isempty(LCF_FV)
        Top_FV = tracks(Id_set(i,2)+1).initialFrame;
        End_FV = tracks(Id_set(i,2)+1).finalFrame;
    else
        Class = M2(tracks(Id_set(i,2)+1).laneletId(1));
        if Class == 1
            Top_FV = tracks(Id_set(i,2)+1).initialFrame;
            End_FV = LCF_FV + tracks(Id_set(i,2)+1).initialFrame-2;
        else
            Top_FV = LCF_FV + tracks(Id_set(i,2)+1).initialFrame-1;
            End_FV = tracks(Id_set(i,2)+1).finalFrame;
        end
    end
    % Lag车辆
    LCF_LV = find(tracks(Id_set(i,3)+1).laneChange==1);
    if isempty(LCF_LV)
        Top_LV = tracks(Id_set(i,3)+1).initialFrame;
        End_LV = tracks(Id_set(i,3)+1).finalFrame;
    else
        Class = M2(tracks(Id_set(i,3)+1).laneletId(1));
        if Class == 1
            Top_LV = tracks(Id_set(i,3)+1).initialFrame;
            End_LV = LCF_LV + tracks(Id_set(i,3)+1).initialFrame-2;
        else
            Top_LV = LCF_LV + tracks(Id_set(i,3)+1).initialFrame-1;
            End_LV = tracks(Id_set(i,3)+1).finalFrame;
        end
    end
    % 确定起始和终止全局帧
    if size(Top_FV,1)~=1||size(Top_LV,1)~=1
        continue;
    end
    if size(End_FV,1)~=1||size(End_LV,1)~=1
        continue;
    end
    Start_Frame(i) = max([Top_EV,Top_FV,Top_LV]);
    End_Frame(i)   = min([End_EV,End_FV,End_LV]);
    if End_Frame(i)-Start_Frame(i)<50
        continue;
    end
    %% 提取EV数据
    LocalStartFrame = Start_Frame(i)-tracks(Id_set(i,1)+1).initialFrame+1;
    LocalEndFrame = End_Frame(i)-tracks(Id_set(i,1)+1).initialFrame+1;
    Total_Frame = End_Frame(i)-Start_Frame(i)+1; % 总共提取的数据帧数
    EV_Data = zeros(Total_Frame,6); %x,y,vx,vy,ax,ay
    Pos1 = repelem(1.875,LC_Frame(i)-Start_Frame(i));
    Pos2 = repelem(-1.875,End_Frame(i)-LC_Frame(i)+1);
    Pos  = [Pos1';Pos2'];
    EV_Data(:,2) = Pos+(3.75./tracks(Id_set(i,1)+1).laneWidth(LocalStartFrame:LocalEndFrame).*tracks(Id_set(i,1)+1).latLaneCenterOffset(LocalStartFrame:LocalEndFrame));
    [Evy,Eay,Evx,Eax] = getVelocityandAcceleration(tracks(Id_set(i,1)+1),LocalStartFrame,LocalEndFrame);
    EV_Data(:,3) = Evx;
    EV_Data(:,4) = Evy;
    EV_Data(:,5) = Eax;
    EV_Data(:,6) = Eay;
    EV_Data(:,1) = getLonPos(tracks(Id_set(i,1)+1),LocalStartFrame,LocalEndFrame);
    EV_Data(:,1) = EV_Data(:,1)-EV_Data(1,1); % 将数据初始化到坐标原点处
    %% 提取FV数据
    LocalStartFrame = Start_Frame(i)-tracks(Id_set(i,2)+1).initialFrame+1;
    LocalEndFrame = End_Frame(i)-tracks(Id_set(i,2)+1).initialFrame+1;
    Total_Frame = End_Frame(i)-Start_Frame(i)+1; % 总共提取的数据帧数
    FV_Data = zeros(Total_Frame,6); %x,y,vx,vy,ax,ay
    Pos1 = repelem(-1.875,End_Frame(i)-Start_Frame(i)+1);
    Pos  = Pos1';
    FV_Data(:,2) = Pos+(3.75./tracks(Id_set(i,2)+1).laneWidth(LocalStartFrame:LocalEndFrame).*tracks(Id_set(i,2)+1).latLaneCenterOffset(LocalStartFrame:LocalEndFrame));
    [Evy,Eay,Evx,Eax] = getVelocityandAcceleration(tracks(Id_set(i,2)+1),LocalStartFrame,LocalEndFrame);
    FV_Data(:,3) = Evx;
    FV_Data(:,4) = Evy;
    FV_Data(:,5) = Eax;
    FV_Data(:,6) = Eay;
    FV_Data(:,1) = getLonPos(tracks(Id_set(i,2)+1),LocalStartFrame,LocalEndFrame);
    FV_Data(:,1) = FV_Data(:,1)-FV_Data(1,1);
    %% 提取LV数据
    LocalStartFrame = Start_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1;
    LocalEndFrame = End_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1;
    Total_Frame = End_Frame(i)-Start_Frame(i)+1; % 总共提取的数据帧数
    LV_Data = zeros(Total_Frame,6); %x,y,vx,vy,ax,ay
    Pos1 = repelem(-1.875,End_Frame(i)-Start_Frame(i)+1);
    Pos  = Pos1';
    LV_Data(:,2) = Pos+(3.75./tracks(Id_set(i,3)+1).laneWidth(LocalStartFrame:LocalEndFrame).*tracks(Id_set(i,3)+1).latLaneCenterOffset(LocalStartFrame:LocalEndFrame));
    [Evy,Eay,Evx,Eax] = getVelocityandAcceleration(tracks(Id_set(i,3)+1),LocalStartFrame,LocalEndFrame);
    LV_Data(:,3) = Evx;
    LV_Data(:,4) = Evy;
    LV_Data(:,5) = Eax;
    LV_Data(:,6) = Eay;
    LV_Data(:,1) = getLonPos(tracks(Id_set(i,3)+1),LocalStartFrame,LocalEndFrame);
    LV_Data(:,1) = LV_Data(:,1)-LV_Data(1,1);
    %% 将三辆车的帧进行对齐，使得抓住其位置关系。提取从Ego车辆进入1992为初始位置，终止位置为max（Ego进入离开2002,如果不到离开2002，就找最大的帧）
    % 全局初始帧在各辆车的局部帧
    S_EV = Start_Frame(i)-tracks(Id_set(i,1)+1).initialFrame+1;
    S_FV = Start_Frame(i)-tracks(Id_set(i,2)+1).initialFrame+1;
    S_LV = Start_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1;
    % 找到同一帧时三辆车在纵向的位置差距
    LaneId_E = tracks(Id_set(i,1)+1).laneletId(S_EV);
    Length_E = tracks(Id_set(i,1)+1).lonLaneletPos(S_EV);
    LaneId_F = tracks(Id_set(i,2)+1).laneletId(S_FV);
    Length_F = tracks(Id_set(i,2)+1).lonLaneletPos(S_FV);
    LaneId_L = tracks(Id_set(i,3)+1).laneletId(S_LV);
    Length_L = tracks(Id_set(i,3)+1).lonLaneletPos(S_LV);
    Dis_EV   = M1(LaneId_E)-Length_E; % Ego车辆距离最迟换道位置距离
    Dis_FV   = M1(LaneId_F)-Length_F; % Front车辆距离最迟换道位置距离
    Dis_LV   = M1(LaneId_L)-Length_L; % Lag车辆距离最迟换道位置距离
    Digit    = find([Dis_EV,Dis_FV,Dis_LV]==max([Dis_EV,Dis_FV,Dis_LV]));
    switch Digit
        case 1
            FV_Data(:,1) = FV_Data(:,1)+(Dis_EV-Dis_FV);
            LV_Data(:,1) = LV_Data(:,1)+(Dis_EV-Dis_LV);
        case 2
            EV_Data(:,1) = EV_Data(:,1)+(Dis_FV-Dis_EV);
            LV_Data(:,1) = LV_Data(:,1)+(Dis_FV-Dis_LV);
        case 3
            EV_Data(:,1) = EV_Data(:,1)+(Dis_LV-Dis_EV);
            FV_Data(:,1) = FV_Data(:,1)+(Dis_LV-Dis_FV);
    end
    Data_EFL(i).EV_Data = EV_Data;
    Data_EFL(i).FV_Data = FV_Data;
    Data_EFL(i).LV_Data = LV_Data;
end
Data_EFL(i).Start_Frame = Start_Frame;
Data_EFL(i).End_Frame = End_Frame;
end

function [Vy,Ay,Vx,Ax] = getVelocityandAcceleration(track,SF,EF)
%% 找到车辆横纵向速度以及加速度
global M2
LC_Frame = find(track.laneChange==1);
Class = M2(track.laneletId(1)); % (0表示左车道，1表示右车道)
if isempty(LC_Frame)&&Class == 1
    Pos1 = repelem(-1.875,track.numFrames);
    Pos  = Pos1';
elseif isempty(LC_Frame)&&Class == 0
    Pos1 = repelem(1.875,track.numFrames);
    Pos  = Pos1';
elseif ~isempty(LC_Frame)&&Class == 0
    Pos1 = repelem(1.875,LC_Frame-1);
    Pos2 = repelem(-1.875,track.numFrames-LC_Frame+1);
    Pos  = [Pos1';Pos2'];
else
    Pos1 = repelem(-1.875,LC_Frame-1);
    Pos2 = repelem(1.875,track.numFrames-LC_Frame+1);
    Pos  = [Pos1';Pos2'];
end
latLaneCenterOffset = Pos+(3.75./track.laneWidth.*track.latLaneCenterOffset);
Velocity = sqrt(track.xVelocity.*track.xVelocity + track.yVelocity.*track.yVelocity);
% 计算车辆的横向速度和加速度（先曲线拟合再求导求导）
t = 0:0.04:0.04*(size(latLaneCenterOffset,1)-1);
p = polyfit(t,latLaneCenterOffset',7);  %使用7次多项式拟合
latLaneCenterOffsetDupty=polyval(p,t);
Vy = diff(latLaneCenterOffsetDupty)./diff(t);
Vy = [Vy(1),Vy];
Ay = diff(Vy)./diff(t);
Ay = [Ay(1),Ay];
Vy = Vy';
Vx = sqrt(Velocity.*Velocity-Vy.*Vy);
Ax = track.lonAcceleration;
Vy = Vy(SF:EF);
Ay = Ay(SF:EF);
Ay = Ay';
Vx = Vx(SF:EF);
Ax = Ax(SF:EF);
end

function Y = getLonPos(track,SF,EF)
%% 计算车辆的纵向位置坐标
% 找到车辆行驶道路段发生变化的帧
LaneletIdChange = track.lonLaneletPos(2:end)-track.lonLaneletPos(1:end-1);
LaneletIdChange = [0;LaneletIdChange];
IdChange = find(LaneletIdChange<0);
laneletLength = zeros(1,size(IdChange,1));
for k=1:size(IdChange,1)
    laneletLength(k) = track.laneletLength(IdChange(k)-1); % 找到各分段道路的长度
end
laneletLength     = [0;laneletLength'];
IdChange = [1;IdChange;size(LaneletIdChange,1)+1];
Y = zeros(size(LaneletIdChange,1),1);
for i=1:size(IdChange,1)-1
    for j = IdChange(i):IdChange(i+1)-1
        Y(j) = track.lonLaneletPos(j)+sum(laneletLength(1:i));
    end
end
Y = Y(SF:EF);
end