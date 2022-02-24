function [pSpotOccupied] = isParkingSpotOccupied(pSpotSize,xData, yData, zData, ptsThreshold)
%UNTITLED Summary of this function goes here
%   Function to determine if one or several static points are located
%   inside the parking spot denoted by pSpotSize


    numPtsInside = 0;
    %ptsThreshold = 2;

    for i = 1:length(xData)
        xInside = (pSpotSize(1) < xData(i)) && ((xData(i) < (pSpotSize(1) + pSpotSize(3))));
        yInside = (pSpotSize(2) < yData(i)) && ((yData(i) < (pSpotSize(2) + pSpotSize(4))));
        if (xInside && yInside)
            numPtsInside = numPtsInside+1;
        end
    end

    if (numPtsInside >= ptsThreshold)
        pSpotOccupied = 1;
    else
        pSpotOccupied = 0;
    end

end

