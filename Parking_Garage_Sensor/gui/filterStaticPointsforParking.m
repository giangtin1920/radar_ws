function [XDataOut, YDataOut, ZDataOut, XData, YData, ZData] = filterStaticPointsforParking(XDataIn, YDataIn, ZDataIn, ...
                                                                         XData, YData, ZData, ...
                                                                         staticPtCloudHistoryNum,constantFrameCountThresh, staticPtCloudHistoryThresh,...
                                                                         snrData,snrThresh)
%filterStaticPointsforParking() SNR and temporal filter on static pts
%   
    temporalFilter = 1;         %enable temporal filter
    snrFilter = 1;              %enable SNR filter


    constantFrameCount = 0;
    XDataInTemp = [];
    YDataInTemp = [];
    ZDataInTemp = [];
    
    if(staticPtCloudHistoryNum < staticPtCloudHistoryThresh)
        XData{1,staticPtCloudHistoryNum} = XDataIn;
        YData{1,staticPtCloudHistoryNum} = YDataIn;
        ZData{1,staticPtCloudHistoryNum} = ZDataIn;
        
        XDataOut = XDataIn;
        YDataOut = YDataIn;
        ZDataOut = ZDataIn;
    else
        XData(1,1:staticPtCloudHistoryNum-2) = XData(1,2:staticPtCloudHistoryNum-1);
        XData{1,staticPtCloudHistoryNum-1} = XDataIn;

        YData(1,1:staticPtCloudHistoryNum-2) = YData(1,2:staticPtCloudHistoryNum-1);
        YData{1,staticPtCloudHistoryNum-1} = YDataIn;

        ZData(1,1:staticPtCloudHistoryNum-2) = ZData(1,2:staticPtCloudHistoryNum-1);
        ZData{1,staticPtCloudHistoryNum-1} = ZDataIn;

        XDataConsistant = double.empty;
        YDataConsistant = double.empty;
        ZDataConsistant = double.empty;

        
        if (snrFilter == 1)
            for i = 1:length(XDataIn)
                if (snrData(i) >= snrThresh)
                    XDataInTemp = [XDataInTemp, XDataIn(1,i)];
                    YDataInTemp = [YDataInTemp, YDataIn(1,i)];
                    ZDataInTemp = [ZDataInTemp, ZDataIn(1,i)];
                end
            end
            XDataIn = XDataInTemp;
            YDataIn = YDataInTemp;
            ZDataIn = ZDataInTemp;
        end
        
       
        if (temporalFilter == 1)
            for i =1:length(XDataIn) %number of points in the current frame
                for j = 1:staticPtCloudHistoryNum-1 %number of cells in staticPtCloudHistory, each cell being 1 frame
                    for k = 1:length(XData{1,j}) %number of pts for that frame
                        if ((XDataIn(1,i) == XData{1,j}(1,k)) && ...
                            (YDataIn(1,i) == YData{1,j}(1,k)) && ...
                            (ZDataIn(1,i) == ZData{1,j}(1,k)))
                            constantFrameCount = constantFrameCount + 1;

                        end
                    end
                end

                if constantFrameCount >= constantFrameCountThresh
                    XDataConsistant = [XDataConsistant ; XDataIn(1,i)];
                    YDataConsistant = [YDataConsistant ; YDataIn(1,i)];
                    ZDataConsistant = [ZDataConsistant ; ZDataIn(1,i)];
                end
                constantFrameCount = 0;
            end

            XDataOut = XDataConsistant;
            YDataOut = YDataConsistant;
            ZDataOut = ZDataConsistant;
        else
            XDataOut = XDataIn;
            YDataOut = YDataIn;
            ZDataOut = ZDataIn;
        end
        %staticPtCloudHistoryNum = 0;
    end
end

