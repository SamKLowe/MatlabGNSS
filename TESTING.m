fid = fopen('GPS_test120s_5_4M.dat','r');  % remember to change the filename to the file you actually want to read

data =  double(fread(fid,[2,1],'*uint32'));
input = (data(1,1)/1.0e9-2.1)'

test= data
I = (data(1,1)/1.0e9-2.1)'
Q = (data(2,:)/1.0e9-2.1)'
fclose(fid);
%stem(z)
scatter(I, Q)