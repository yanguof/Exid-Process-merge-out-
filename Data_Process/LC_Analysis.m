function [Data_Start,Data_Cross,Class] = LC_Analysis(Id_set,LC_Frame,LC_Start,tracks)
%% 输出换道点换道车辆距离换道终止点（即离开1998路段）的距离统计,此时的Ego侧向车速，车辆类型，与后车的速差，TTC，THW，DHW

% 最迟换道点距各段的距离
keySet = {1990,1991,1994,1995,1992,1996,2155,2156,2222,2223,2157,2158,2010,2001,1998,2002 2228 2003};
valueSet = [274.64 243.85 274.11 244.34 216.24 216.24 193.91 193.91 170.73 170.73 141.37 141.37 8.06 8.06 4.94 4.94 -17.66 -33.52];
M = containers.Map(keySet,valueSet);
Class = {};
for i=1:size(Id_set,1)
    Pos1 = repelem(1.875,LC_Frame(i)-tracks(Id_set(i,1)+1).initialFrame);
    Pos2 = repelem(-1.875,tracks(Id_set(i,1)+1).numFrames-LC_Frame(i)+tracks(Id_set(i,1)+1).initialFrame);
    Pos  = [Pos1';Pos2'];
    EV_Data = Pos+(3.75./tracks(Id_set(i,1)+1).laneWidth.*tracks(Id_set(i,1)+1).latLaneCenterOffset);
    [Evy,Eay,Evx,~] = getVelocityandAcceleration(EV_Data,tracks(Id_set(i,1)+1));
    %% 换道点
    YVel_EV(i) = Evy(LC_Frame(i)-tracks(Id_set(i,1)+1).initialFrame+1);
    XVel_EV(i) = Evx(LC_Frame(i)-tracks(Id_set(i,1)+1).initialFrame+1);
    YAcc_EV(i) = Eay(LC_Frame(i)-tracks(Id_set(i,1)+1).initialFrame+1);
    Acc_LV(i) = tracks(Id_set(i,3)+1).lonAcceleration(LC_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1);
    TTC(i)   = tracks(Id_set(i,3)+1).leadTTC(LC_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1);
    THW(i)   = tracks(Id_set(i,3)+1).leadTHW(LC_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1);
    DHW(i)   = tracks(Id_set(i,3)+1).leadDHW(LC_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1);
    RV(i)    = tracks(Id_set(i,3)+1).leadDV(LC_Frame(i)-tracks(Id_set(i,3)+1).initialFrame+1); %相对车速（后车-前车）
    Class{i} = tracks(Id_set(i,1)+1).class;
    %% 换道初始帧（全局）
    DisE = M(tracks(Id_set(i,1)+1).laneletId(LC_Start(i)-tracks(Id_set(i,1)+1).initialFrame+1))-tracks(Id_set(i,1)+1).lonLaneletPos(LC_Start(i)-tracks(Id_set(i,1)+1).initialFrame+1);
    DisF = M(tracks(Id_set(i,2)+1).laneletId(LC_Start(i)-tracks(Id_set(i,2)+1).initialFrame+1))-tracks(Id_set(i,2)+1).lonLaneletPos(LC_Start(i)-tracks(Id_set(i,2)+1).initialFrame+1);
    DisL = M(tracks(Id_set(i,3)+1).laneletId(LC_Start(i)-tracks(Id_set(i,3)+1).initialFrame+1))-tracks(Id_set(i,3)+1).lonLaneletPos(LC_Start(i)-tracks(Id_set(i,3)+1).initialFrame+1); 
    GapLE(i) = DisL-DisE;
    GapFE(i) = DisE-DisF;
    DetaVL(i)= tracks(Id_set(i,1)+1).lonVelocity(LC_Start(i)-tracks(Id_set(i,1)+1).initialFrame+1)-tracks(Id_set(i,3)+1).lonVelocity(LC_Start(i)-tracks(Id_set(i,3)+1).initialFrame+1);
    DetaVF(i)= tracks(Id_set(i,1)+1).lonVelocity(LC_Start(i)-tracks(Id_set(i,1)+1).initialFrame+1)-tracks(Id_set(i,2)+1).lonVelocity(LC_Start(i)-tracks(Id_set(i,2)+1).initialFrame+1);
    Distance(i) = DisE;

    %% 换道时间（Ts-Tc）
    LCTime(i) = 0.04*(LC_Frame(i)-LC_Start(i));
    Ts_local = LC_Start(i)-tracks(Id_set(i,1)+1).initialFrame+1;
    Tc_local = LC_Frame(i)-tracks(Id_set(i,1)+1).initialFrame+1;
    LCDistance(i) = tracks(Id_set(i,1)+1).traveledDistance(Tc_local)-tracks(Id_set(i,1)+1).traveledDistance(Ts_local);
    Velocity_EVS(i) = tracks(Id_set(i,1)+1).lonVelocity(Ts_local);
    Velocity_EVC(i) = tracks(Id_set(i,1)+1).lonVelocity(Tc_local);
end
Data_Start = [GapLE',DetaVL',GapFE',DetaVF',Distance',Velocity_EVS',LCTime',LCDistance'];
Data_Cross = [XVel_EV',YVel_EV',YAcc_EV',Acc_LV',TTC',THW',DHW',RV',Velocity_EVC'];
end

function [Vy,Ay,Vx,Ax] = getVelocityandAcceleration(latLaneCenterOffset,track)
%% 找到车辆横纵向速度以及加速度
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
end
