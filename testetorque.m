pos = [0 0 0 0 0 0 0 0 -300 0 0 0];
vel = 255*ones(size(pos));
posfinal = [];
velfinal = [];

for i = 1: (12*100*5)
    posfinal = [posfinal pos];
    velfinal = [velfinal vel];
end

final = [posfinal; velfinal];

fileID = fopen('teste/first.bin','w');
fwrite(fileID,final,'int16','l');
fclose(fileID);

fileID = fopen('teste/forward.bin','w');
fwrite(fileID,final,'int16','l');
fclose(fileID);

fileID = fopen('teste/last.bin','w');
fwrite(fileID,final,'int16','l');
fclose(fileID);
