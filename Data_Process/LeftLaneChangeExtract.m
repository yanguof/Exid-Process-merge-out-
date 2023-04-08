function [Id_set,LC_Frame] = LeftLaneChangeExtract(tracks)
% 提取出来左换道车辆的id以及其目标车道前车后车id,以及换道点所处的全局帧编号
Id_set = [];
LC_Frame = [];
for i=1:size(tracks,2)
    LanechangeFrame_local = find(tracks(i).laneChange==1); %找到换道点及其所处的位置
    if ismember(1996,tracks(i).laneletId) && size(LanechangeFrame_local,1)==1
        EV = i-1;
        FV = tracks(i).leadId(LanechangeFrame_local);
        LV = tracks(i).rearId(LanechangeFrame_local);
        if FV~=-1 && LV~=-1
            LanechangeFrame_global = LanechangeFrame_local+tracks(i).initialFrame-1;
            if tracks(LV+1).leadTHW(LanechangeFrame_global+1-tracks(LV+1).initialFrame)<=2
                Id_set = [Id_set;EV,FV,LV];
                LC_Frame = [LC_Frame;LanechangeFrame_global];
            end
        end
    else
    end
end
end