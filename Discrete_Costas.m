%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%Trunnel GNSS Reciever Dicrete Costas Loop Simulation
%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;


%%% Adjustment Constants
SAMPLE_FREQUENCY = 10230000;
SAMPLE_RATE_MULT = 2; % default rate is 5 samples per period


%%%%%Open input file snr3/snr100
filename='snr100.txt';

%Signal Generation Constants
SIG2NOISE_RATIO = 7; % decrease to add more noise
INTEGRAL_ITERATIONS = 7; %increase to increase the effect of the low pass filter
PHI_ADJUST = 3; %must do some extra stuff to it to turn it into a cos wave
PHI_INIT = ((0-PHI_ADJUST)-1)-5;   % this number * pi/2 some weird math to line up right
START_PHI = (pi/(5*SAMPLE_RATE_MULT)) * PHI_INIT; %used to initialize 
STREAM_SAMPLES = 20100; %will eventually auto detect this
SAMPLES = 2000; % amount of samples to use
EXTRA_SAMPLES = (5*SAMPLE_RATE_MULT) - (5+PHI_INIT); %used for phase offset

%C/A arrays to use for various samples
%in future can add a function that builds this
goldcode2=[0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1];
cw = [0 1]; %20
%cw = [1 0 1 0 1 0 1 1 0 0]; %100
%cw = [1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0]; % 500
rand_bit = 0;

for l = (1 : 50)
    rand_bit = randi(2,1,1);
    rand_bit = rand_bit - 1;
    cw = [cw rand_bit];
end
cw = [cw goldcode2];
for l = (1 : 50)
    rand_bit = randi(2,1,1);
    rand_bit = rand_bit - 1;
    cw = [cw rand_bit];
end

x = cw';

bits = length(cw);

%%%%% Output Bit Decision Variables %%%%%%%%%%%%%%%%%%%%%
EARLY_LATE_WIDTH = 3; % This will let us change the scope of our early late 
                      % detector
pSamp = 5; %prompt sample variable
eSamp = 5 - EARLY_LATE_WIDTH; %early sample variable
lSamp = 5 + EARLY_LATE_WIDTH; %late sample variable


% ---------------- INPUT SIGNAL GENERATION ------------------------------
%create a square wave to multiply with the sine carrier
chip = ones([1,(5 *SAMPLE_RATE_MULT) ]);

cwSamp = [];


for l=(1:(EXTRA_SAMPLES))
    cwSamp = [cwSamp 1];
end
for k=(1:length(cw))
    if (cw(k) == 0)
        cwSamp = [cwSamp -chip];
    else
        cwSamp = [cwSamp chip];
    end;
end;




%constructing the carrier wave using the dsp.SineWave function
carrier = dsp.SineWave();
carrier.Frequency = SAMPLE_FREQUENCY;
carrier.Amplitude = 1;
carrier.PhaseOffset = START_PHI+5;
carrier.SamplesPerFrame = 5*SAMPLE_RATE_MULT*bits + EXTRA_SAMPLES;
carrier.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*10;
carrier.OutputDataType = 'single';
cwWave = step(carrier);


%constructing the message signal
cwSamp = (cwSamp)';
z = cwWave;
WaveLen = length(cwWave);
SampLen = length(cwSamp);

%only use these to generate new input files
input2 = cwWave.*cwSamp;
input_clean = input2;
input2 = awgn(input2, SIG2NOISE_RATIO);

s1 = zeros(SAMPLES);
s2 = zeros(SAMPLES);

fileID=fopen(filename, 'r');%open file


%%build costas cosine and sine waves

sinMult = dsp.SineWave();
sinMult.Frequency = SAMPLE_FREQUENCY;
sinMult.Amplitude = 1;
sinMult.PhaseOffset = 0;
sinMult.SamplesPerFrame =  5*SAMPLE_RATE_MULT*bits;
sinMult.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*10;
sinMult.OutputDataType = 'single';

cosMult = dsp.SineWave();
cosMult.Frequency = SAMPLE_FREQUENCY;
cosMult.Amplitude = 1;
cosMult.PhaseOffset = pi/2;
cosMult.SamplesPerFrame = 5*SAMPLE_RATE_MULT*bits;
cosMult.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*10;
cosMult.OutputDataType = 'single';

sinWave = step(sinMult);
cosWave = step(cosMult);

corrected_flag = 0;

%Costas Integrator variables
y1 = zeros(SAMPLES);
y2 = zeros(SAMPLES);


%%%% PID Values

setpoint = 0;
error = 0;
previous_error = 0;
integral = 0;
derivative = 0;
output = 0;
Ki = 3000000;   % divided by 1.024 Million (dt)
Kd = .0000000001; % multiplied by 1.024 Million (dt)
Kp = .000000000002;
dt = 1/(SAMPLE_FREQUENCY*SAMPLE_RATE_MULT);

%phase variable
phi = 0;

%xor variables
shift=[];
shiftindex=0;
curr_samp=0;
n=0;
xoredbits=[];
lengthofgoldcode=length(goldcode2);
next_curr_samp=0;
xoredgraph=[];
early_bit = 0;
prompt_bit = 0;
late_bit = 0;
xoredmag=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%MAIN LOOP
% the range is weird because we need to make room for the phase offset
% for inc = (ceil(SAMPLES / 10):SAMPLES - ceil(SAMPLES / 10))
% for inc = (20*SAMPLE_RATE_MULT:SAMPLES - (20*SAMPLE_RATE_MULT)) 

for i = (1:EXTRA_SAMPLES)
   x=fscanf(fileID,'%f',1);
   
end


for inc = (30:STREAM_SAMPLES)
    
   %Read in data from ADC comment out if using code generated input
   input(inc)= fscanf(fileID,'%f',1);
   
% -----------------------LOOP FILTER (CONTROL LOOP)-----------------------
%PID controller

% Using style from Wikipedia https://en.wikipedia.org/wiki/PID_controller
% previous_error = 0
% integral = 0 
% start:
%   error = setpoint - measured_value
%   integral = integral + error*dt
%   derivative = (error - previous_error)/dt
%   output = Kp*error + Ki*integral + Kd*derivative
%   previous_error = error
%   wait(dt)
%   goto start
    
    pidY1 = y1(inc-1);    
    pidY2 = y2(inc-1);
    error = pidY1*pidY2*2;
    
    errors(inc) = error;
    integral = integral + (error * dt);
    derivative = (error - previous_error)/dt;
    output = (Kp*error + Ki*integral + Kd*derivative);
    previous_error = error;
    phi = round(output);    
    
    outputs(inc) = output;
    
      
    phis(inc) = phi;
    errors(inc) = error;
    
   
% -----------------------MULTIPLYING BY SIN AND COS------------------------
    %%set new phi to waves 

    
    sinArray = input(inc)*sinWave(mod(inc, 20)+phi+210);
    cosArray = input(inc)*cosWave(mod(inc, 20)+phi+210);
        
    
    sinElement = sinArray(1);
    cosElement = cosArray(1);
    
    s1(inc) = sinElement;
    s2(inc) = cosElement;
 

% -----------------------LOW PASS FILTER-----------------------------------
    if inc<=INTEGRAL_ITERATIONS
%  If sample index is less than 100 (Tc/Ts) then we sum available previous
%  samples divided by the number of samples
        for inc2=1:INTEGRAL_ITERATIONS
            y1(inc) = y1(inc) + s1(inc2);
            y2(inc) = y2(inc) + s2(inc2);
        end
        y1(inc) = y1(inc) / INTEGRAL_ITERATIONS;
        y2(inc) = y2(inc) / INTEGRAL_ITERATIONS;
    else
% Summing recent previous 100 (Tc/Ts) values        
        for inc2 = inc-(INTEGRAL_ITERATIONS-1):inc
            y1(inc) = y1(inc) + s1(inc2);
            y2(inc) = y2(inc) + s2(inc2);
        end
        y1(inc) = y1(inc) / INTEGRAL_ITERATIONS;
        y2(inc) = y2(inc) / INTEGRAL_ITERATIONS;
    end 
    
    
    
    

% -----------------------Output Bit Creation------------------------------------   
%This block will take which sample the early late detector wants and take
%the sign of it to send as a bit to the early, late, and punctual xor 
%shift registers.
    y = inc;
    x = mod(inc, (5 * SAMPLE_RATE_MULT));
    z = (phi + EARLY_LATE_WIDTH);
    
    %there is a problem with phi changing at the same time as when the
    %increment gets to the correct value. For now phi gets assigned to
    %phase to make sure that we capture the bit. This only updates the
    %phase at the beginning of each chip
    if(mod(inc, (5 * SAMPLE_RATE_MULT)) == 0)
        phase = phi;
    end
   
    if(mod(inc, (5 * SAMPLE_RATE_MULT)) == (phase + EARLY_LATE_WIDTH))
        next_curr_samp = inc;
        early_bit = sign(y1(inc - phase - EARLY_LATE_WIDTH));
        prompt_bit = sign(y1(inc - phase));
        late_bit = sign(y1(inc));
    end 
    
    
% -----------------------Xor block------------------------------------   

    if(curr_samp ~= next_curr_samp)
    curr_samp=next_curr_samp;
    %shift reg
    %for shiftindex = n+1:n+100%starting smaller
    shift=[shift prompt_bit];%shift in 100 bits to test.
    %end
    %shift through previous 100 samples.
    if length(shift)>lengthofgoldcode+1
        for shiftindex=1:lengthofgoldcode                
            if shift(length(shift)-shiftindex+1) ~= goldcode2(lengthofgoldcode-shiftindex+1)
            %if (xor(shift(length(shift)-shiftindex+1), goldcode2(lengthofgoldcode-shiftindex+1)) == 1)
                xoredbits(shiftindex)=1;
            else
                xoredbits(shiftindex)=-1;
            end
            %xoredbits(shiftindex) = xor(shift(length(shift)-shiftindex+1), goldcode2(lengthofgoldcode-shiftindex+1));
            xoredmag = xoredmag+ xoredbits(shiftindex);
        end
        xoredgraph =[xoredgraph xoredmag];
        xoredmag=0;
     end
    end

end



% -----------------------PLOTS------------------------------------
subplot(2,2,1)
%figure(1)
%stem(input_clean(1701:1850));
%stem(input(1:400));
plot(xoredgraph);
title('Xor Magnitude');
xlabel('Chip Number');
ylabel('Magnitude');
subplot(2,2,2);
%figure(2)
stem(input(300:900));
title('Input');
xlabel('time in sample intervals');
ylabel('Amplitude');
subplot(2,2,3);
%figure(3)
%stem(y1(1701:1850));
stem(y1(300:900));
title('Costas Output Waveform');
xlabel('time in sample intervals');
ylabel('Amplitude');
subplot(2,2,4);
plot(phis(1:length(phis)-((20*SAMPLE_RATE_MULT))))
hold on;
plot(errors(1:length(phis)-((20*SAMPLE_RATE_MULT))))
hold on;
plot(outputs(1:length(phis)-((20*SAMPLE_RATE_MULT))))
hold off;
title('Phase/Error of the System');
xlabel('time in sample intervals');
ylabel('phi');

% print out the final phase
final_phi = phis(length(phis)-((20*SAMPLE_RATE_MULT))) % * pi/5




fclose(fileID);

 
 



