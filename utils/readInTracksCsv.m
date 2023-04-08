function [tracks] = readInTracksCsv(filename, filenameStatic, filenameRecording)
% Read the csv and convert it into a table
csvData = readtable(filename, 'Delimiter', ',');
csvDataStatic = readtable(filenameStatic, 'Delimiter', ',');
csvRecording = readtable(filenameRecording, 'Delimiter', ',');
% Group by track id
G = findgroups(csvData(:, 2));
groupedById = splitapply(@(varargin)(varargin), csvData, G);

% Build a map containing the track id and its corresponding index in the 
% static tracks csv file. This enables fast access to the information when
% creating the final tracks struct.
idList = {};
indexList = [];
for iRow = 1:size(csvDataStatic, 1)
%     iTrackId = csvDataStatic.id(iRow);
    TrackId = csvDataStatic.trackId(iRow);
    idList = [idList; TrackId];
    indexList = [indexList; iRow];
end
staticMap = containers.Map(idList,indexList);

% Initialize tracks 
tracks = {};

% Initialize constant variables
trackIndex = 1;

% Iterate over the whole table
for iRow = 1:size(groupedById, 1)
    iTrack = groupedById(iRow, :);
    iTrackIds = cell2mat(iTrack(2));
    currentId = iTrackIds(1);
    
    tracks(trackIndex).id = currentId;
    tracks(trackIndex).frame = cell2mat(iTrack(3));
    tracks(trackIndex).bbox = [cell2mat(iTrack(5)) ...
                               cell2mat(iTrack(6)) ... 
                               cell2mat(iTrack(7)) ...
                               cell2mat(iTrack(8)) ...
                               cell2mat(iTrack(9))];
    tracks(trackIndex).xVelocity = cell2mat(iTrack(10));
    tracks(trackIndex).yVelocity = cell2mat(iTrack(11));
    tracks(trackIndex).xAcceleration = cell2mat(iTrack(12));
    tracks(trackIndex).yAcceleration = cell2mat(iTrack(13));
    tracks(trackIndex).lonVelocity = cell2mat(iTrack(14));
    tracks(trackIndex).latVelocity = cell2mat(iTrack(15));
    tracks(trackIndex).lonAcceleration = cell2mat(iTrack(16));
    tracks(trackIndex).latAcceleration = cell2mat(iTrack(17));
    tracks(trackIndex).traveledDistance = cell2mat(iTrack(18));
    tracks(trackIndex).latLaneCenterOffset = cell2mat(iTrack(19));
    tracks(trackIndex).laneWidth = cell2mat(iTrack(20));
    tracks(trackIndex).laneletId = cell2mat(iTrack(21));
    tracks(trackIndex).laneChange = cell2mat(iTrack(22));
    tracks(trackIndex).lonLaneletPos = cell2mat(iTrack(23));
    tracks(trackIndex).laneletLength = cell2mat(iTrack(24));
    tracks(trackIndex).leadDHW = cell2mat(iTrack(25));
    tracks(trackIndex).leadDV = cell2mat(iTrack(26));
    tracks(trackIndex).leadTHW = cell2mat(iTrack(27));
    tracks(trackIndex).leadTTC = cell2mat(iTrack(28));
    tracks(trackIndex).leadId = cell2mat(iTrack(29));
    tracks(trackIndex).rearId = cell2mat(iTrack(30));
    tracks(trackIndex).leftLeadId = cell2mat(iTrack(31));
    tracks(trackIndex).leftRearId = cell2mat(iTrack(32));
    tracks(trackIndex).leftAlongsideId = cell2mat(iTrack(33));
    tracks(trackIndex).rightLeadId = cell2mat(iTrack(34));
    tracks(trackIndex).rightRearId = cell2mat(iTrack(35));
    tracks(trackIndex).rightAlongsideId = cell2mat(iTrack(36));
    tracks(trackIndex).odrRoadId = cell2mat(iTrack(37));
    tracks(trackIndex).odrSectionNo = cell2mat(iTrack(38));
    tracks(trackIndex).odrLaneId = cell2mat(iTrack(39));
    
    csvStaticIndex = staticMap(currentId);
    tracks(trackIndex).initialFrame = csvDataStatic.initialFrame(csvStaticIndex);
    tracks(trackIndex).finalFrame = csvDataStatic.finalFrame(csvStaticIndex);
    tracks(trackIndex).numFrames = csvDataStatic.numFrames(csvStaticIndex);
    extractedClass = csvDataStatic.class(csvStaticIndex);
    tracks(trackIndex).class = extractedClass{1};
    tracks(trackIndex).frameRate = csvRecording.frameRate;
    tracks(trackIndex).latLocation = csvRecording.latLocation;
    tracks(trackIndex).lonLocation = csvRecording.lonLocation;
    tracks(trackIndex).xUtmOrigin = csvRecording.xUtmOrigin;
    tracks(trackIndex).yUtmOrigin = csvRecording.yUtmOrigin;
    tracks(trackIndex).orthoPxToMeter = csvRecording.orthoPxToMeter;
    
    % Increment the internal track index 
    trackIndex = trackIndex + 1;
end
end
