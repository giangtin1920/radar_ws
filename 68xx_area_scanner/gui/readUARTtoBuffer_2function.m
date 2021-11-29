%% main - parse UART and update plots

[newframe, bytesBuffer, bytesBufferLen, numFramesAvailable,validFrame] = parseBytes_AS(bytesBuffer, bytesBufferLen, READ_MODE);
% word = [1 256 65536 16777216]';
% nfp = newframe.packet(17:20)'
% tt= sum(nfp .* word)