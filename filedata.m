filename = 'test.dat';

fileID = fopen(filename, 'r');

A = fread(fileID, [2 200], 'single');

y = A(1, :);

f = fft(y);

plot(abs(f))

fclose(fileID);

