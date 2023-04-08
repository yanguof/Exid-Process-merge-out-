function [Id_set,LC_Cross,LC_Start] = RightLaneChangeExtract(tracks)
% 提取出来右换道车辆的id以及其目标车道前车后车id,以及换道点所处的全局帧编号
Id_set = [];
LC_Cross = [];
LC_Start = [];
% 匹配目标车道后车在哪一个车道
keySet = {1990,1991,1992,2155,2222,2157,2010,1998,2002,2001,2158,2223,2156,1996,1995 1994};
valueSet = [0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1];
M = containers.Map(keySet,valueSet);
for i=1:size(tracks,2)
    LanechangeFrame_local = find(tracks(i).laneChange==1); %找到换道点及其所处的位置
    if ismember(1992,tracks(i).laneletId) && size(LanechangeFrame_local,1)==1
        EV = i-1;
        FV = tracks(i).leadId(LanechangeFrame_local);
        LV = tracks(i).rearId(LanechangeFrame_local);
        if FV~=-1 && LV~=-1
            %% 提取EV数据
            LanechangeFrame_global = LanechangeFrame_local+tracks(i).initialFrame-1;
            Pos1 = repelem(1.875,LanechangeFrame_local-1);
            Pos2 = repelem(-1.875,tracks(EV+1).numFrames-LanechangeFrame_local+1);
            Pos  = [Pos1';Pos2'];
            EV_Data = Pos+(3.75./tracks(EV+1).laneWidth.*tracks(EV+1).latLaneCenterOffset); %x,y,vx,vy,ax,ay
            [Evy,~,~,~] = getVelocityandAcceleration(EV_Data,tracks(EV+1));
            LC_start1 = find(Evy>=-0.34);
            LC_start2 = find(LC_start1<LanechangeFrame_local);
            LC_start  = LC_start1(LC_start2(end))+1;
            LCStartFrame_global = LC_start+tracks(i).initialFrame-1;
            if LCStartFrame_global+1<=tracks(LV+1).initialFrame % 判断Ego车辆开始换道时，后车是否存在及其存在的位置
                continue;
            else
                value = M(tracks(LV+1).laneletId(LCStartFrame_global-tracks(LV+1).initialFrame+1));
            end
            if tracks(LV+1).leadTHW(LanechangeFrame_global+1-tracks(LV+1).initialFrame)<=2 && value==1 && tracks(LV+1).leadDHW(LCStartFrame_global+1-tracks(LV+1).initialFrame)<=120
                Id_set = [Id_set;EV,FV,LV];
                LC_Cross = [LC_Cross;LanechangeFrame_global];
                LC_Start = [LC_Start;LCStartFrame_global];
            end
        end
    else
    end
end
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