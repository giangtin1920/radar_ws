function [hWarningSpot] = drawWarningSpot(parentFigure, coordMatrix)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

hWarningSpot.spot1 = rectangle('parent', parentFigure, 'Position',coordMatrix, 'edgecolor', [0.5 0.5 0], ...
                                'facecolor', [0.2,0.2,0], 'linewidth', 3, 'linestyle', '--');
end

