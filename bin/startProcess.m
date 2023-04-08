clc; clear;
% Name of the video
videoString_set = {'19' '20' '21' '22' '23' '24' '25' '26' '27' '28' '29' '30' '31' '32' '33' '34' '35' '36' '37' '38' };
for k = 1:size(videoString_set,2)
    videoString = videoString_set{k};
    % Define path of background image. Can be empty.
    backgroundImagePath = sprintf('data/%s_background.png', videoString);
    % Read tracks by using tracks file and static tracks file.
    tracksFilename = sprintf('data/%s_tracks.csv', videoString);
    tracksStaticFilename = sprintf('data/%s_tracksMeta.csv', videoString);
    tracksRecordingFilename = sprintf('data/%s_recordingMeta.csv', videoString);
    tracks = readInTracksCsv(tracksFilename, tracksStaticFilename,tracksRecordingFilename); %存储所有信息
    [Id_set,LC_Frame,LC_Start] = RightLaneChangeExtract(tracks); % 提取出来右换道车辆的id以及其目标车道前车后车id,以及换道点所处的全局帧编号
    if size(Id_set,1)~=0
        %% 统计换道初始点和越过车道线点的相关参数
        [Data_Start,Data_Cross,Class] = LC_Analysis(Id_set,LC_Frame,LC_Start,tracks); % 输出换道点换道车辆距离换道终止点（即离开1998路段）的距离统计,此时的Ego侧向车速，
        % 车辆类型，与后车的速差，TTC，THW，DHW等等
        Analysis_Data.Data_Start = Data_Start;
        Analysis_Data.Data_Cross = Data_Cross;
        Analysis_Data.Class = Class';
        Analysis_Data.Id_set = Id_set';
        Analysis_Data.LC_Frame = LC_Frame';
        Analysis_Data.LC_Start = LC_Start';
        filename =  strcat('Analysis_Data',   num2str(k),  '.mat');
        save(filename, 'Analysis_Data');
%         %% 提取三辆车的相应参数以方便决策框架的验证
%         Data_EFL = DataExtract(Id_set,LC_Frame,tracks); % 输出三辆车的运动信息(E,F,L)
%         filename =  strcat('Scenario',   num2str(k),  '.mat');
%         save(filename, 'Data_EFL');
    else
    end
end