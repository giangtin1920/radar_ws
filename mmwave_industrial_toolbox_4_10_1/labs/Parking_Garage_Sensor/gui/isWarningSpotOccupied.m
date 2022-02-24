function [wSpotOccupied] = isWarningSpotOccupied(pWarningSpotSize,xData, yData, zData, ptsThreshold)
%UNTITLED Summary of this function goes here
%   Function to determine if one or several static points are located
%   inside the parking spot denoted by pSpotSize


    numPtsInside = 0;
    %ptsThreshold = 2;

    for i = 1:length(xData)
        xInside = (pWarningSpotSize(1) < xData(i)) && ((xData(i) < (pWarningSpotSize(1) + pWarningSpotSize(3))));
        yInside = (pWarningSpotSize(2) < yData(i)) && ((yData(i) < (pWarningSpotSize(2) + pWarningSpotSize(4))));
        if (xInside && yInside)
            numPtsInside = numPtsInside+1;
        end
    end

    if (numPtsInside >= ptsThreshold)
        wSpotOccupied = 1;
    else
        wSpotOccupied = 0;
    end

end

