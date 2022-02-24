function [parkStruct] = parseParkingSetup(parkStruct, filepath,filename)
%parseParkingSetup parse configuration of parking spaces and filter params
%   Detailed explanation goes here

fullPathName = strcat(filepath, filename);

%opts = detectImportOptions(fullPathName);
parkingFile = readtable(fullPathName);
%print (parkingFile);

parkingFile = readtable(fullPathName);
%print (parkingFile);
for i=1:height(parkingFile)
    switch string(parkingFile.param(i))
        case 'constantFrameCountThresh'
            parkStruct.constantFrameCountThresh = parkingFile.value1(i);
        case 'staticPtCloudHistoryThresh'
            parkStruct.staticPtCloudHistoryThresh = parkingFile.value1(i);
        case 'numSpots'
            parkStruct.numSpots = parkingFile.value1(i);
        case 'parkingSpotSize1'
            parkStruct.hparkingSpotSize(1,:) = [parkingFile.value1(i),parkingFile.value2(i),parkingFile.value3(i),parkingFile.value4(i)];
        case 'parkingSpotSize2'
            parkStruct.hparkingSpotSize(2,:) = [parkingFile.value1(i),parkingFile.value2(i),parkingFile.value3(i),parkingFile.value4(i)];
        case 'parkingSpotSize3'
            parkStruct.hparkingSpotSize(3,:) = [parkingFile.value1(i),parkingFile.value2(i),parkingFile.value3(i),parkingFile.value4(i)];
        case 'parkingSpotSize4'
            parkStruct.hparkingSpotSize(4,:) = [parkingFile.value1(i),parkingFile.value2(i),parkingFile.value3(i),parkingFile.value4(i)];
        case 'pSpotOccupiedPtsThresh'
            parkStruct.pSpotOccupiedPtsThresh = parkingFile.value1(i);
        case 'pSpotOccupiedHyst'
            parkStruct.pSpotOccupiedHyst = parkingFile.value1(i);
        case 'snrThresh'
            parkStruct.snrThresh = parkingFile.value1(i);
        case 'displayWarningSpot'
            parkStruct.displayWarningSpot = parkingFile.value1(i);
        case 'hWarningSpotSize'
            parkStruct.hWarningSpotSize(1,:) = [parkingFile.value1(i),parkingFile.value2(i),parkingFile.value3(i),parkingFile.value4(i)];
        case 'wSpotOccupiedPtsThresh'
            parkStruct.wSpotOccupiedPtsThresh = parkingFile.value1(i);
        case 'wSpotOccupiedHyst'
            parkStruct.wSpotOccupiedHyst = parkingFile.value1(i);
    end
end

end

