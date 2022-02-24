function [hParkingSpot] = drawParkingSpot(parentFigure, coordMatrix)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

hParkingSpot.spot1 = rectangle('parent', parentFigure, 'Position',coordMatrix, 'edgecolor', [0 1 0], ...
                                'facecolor', [0,0.2,0], 'linewidth', 3);
end

