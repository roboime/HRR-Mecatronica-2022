P = 4095*ones(1,12*100*5);
V = 200*ones(size(P));
P0 = zeros(size(P));
M = [P P0; V V];
fileID = fopen('forward.bin','w');
fwrite(fileID,M,'int16','l');
fclose(fileID);
