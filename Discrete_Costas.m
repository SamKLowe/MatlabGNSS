%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%Trunnel GNSS Reciever Dicrete Costas Loop Simulation
%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;

%%%%%%%%%%%%%%%%%%%%%%%% Adjustments
SIG2NOISE_RATIO = 0; % decrease to add more noise
INTEGRAL_ITERATIONS = 7; %increase to increase the effect of the low pass filter
SAMPLE_RATE_MULT = 2; % default rate is 10 samples per period
PHI_ADJUST = 3; %must do some extra stuff to it to turn it into a cos wave
PHI_INIT = PHI_ADJUST - 5;   % this number * pi/2
START_PHI = (pi/(5*SAMPLE_RATE_MULT)) * PHI_INIT; %used to initialize 

SAMPLES = 2000; % amount of samples to use
EXTRA_SAMPLES = (5*SAMPLE_RATE_MULT) - (5+PHI_INIT); %used for phase offset

%C/A arrays to use for various samples
%in future can add a function that builds this
%cw = [0 1]; %20
%cw = [1 0 1 0 1 0 1 1 0 0]; %100
cw = [1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0]; % 500

BITS = length(cw);


% -----------------------INPUT SIGNAL------------------------------------
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


SAMPLE_FREQUENCY = 10230000;

%constructing the carrier wave using the dsp toolbox
carrier = dsp.SineWave();
carrier.Frequency = SAMPLE_FREQUENCY;
carrier.Amplitude = 1;
carrier.PhaseOffset = START_PHI+5;
carrier.SamplesPerFrame = 5*SAMPLE_RATE_MULT*BITS + EXTRA_SAMPLES;
carrier.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*10;
carrier.OutputDataType = 'single';
cwWave = step(carrier);


%%%%%%%%%%constructing the message signal
cwSamp = (cwSamp)';
z = cwWave;
WaveLen = length(cwWave);
SampLen = length(cwSamp);

input = cwWave.*cwSamp;
input_clean = input;

input = awgn(input, SIG2NOISE_RATIO);
Mag_Noise = 1/db2mag((SIG2NOISE_RATIO))


s1 = zeros(SAMPLES);
s2 = zeros(SAMPLES);

%%build cosin and sin waves

sinMult = dsp.SineWave();
sinMult.Frequency = SAMPLE_FREQUENCY;
sinMult.Amplitude = 1;
sinMult.PhaseOffset = 0;
sinMult.SamplesPerFrame =  5*SAMPLE_RATE_MULT*BITS;
sinMult.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*10;
sinMult.OutputDataType = 'single';

cosMult = dsp.SineWave();
cosMult.Frequency = SAMPLE_FREQUENCY;
cosMult.Amplitude = 1;
cosMult.PhaseOffset = pi/2;
cosMult.SamplesPerFrame = 5*SAMPLE_RATE_MULT*BITS;
cosMult.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*10;
cosMult.OutputDataType = 'single';



sinWave = step(sinMult);
cosWave = step(cosMult);



corrected_flag = 0;

%integrator variables
y1 = zeros(SAMPLES);
y2 = zeros(SAMPLES);


%%%% PID Values

setpoint = 0;
error = 0;
previous_error = 0;
integral = 0;
derivative = 0;
output = 0;
Ki = 4000000;   % divided by 1.024 Million (dt)
Kd = .00000002; % multiplied by 1.024 Million (dt)
Kp = 2;
dt = 1/(SAMPLE_FREQUENCY*SAMPLE_RATE_MULT);

%phase variable
phi = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%MAIN LOOP
% the range is weird because we need to make room for the phase offset
% for inc = (ceil(SAMPLES / 10):SAMPLES - ceil(SAMPLES / 10))
for inc = (20*SAMPLE_RATE_MULT:SAMPLES - (20*SAMPLE_RATE_MULT))    
    
   
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
    error = pidY1*pidY2;
    
    errors(inc) = error;
    integral = integral + (error * dt);
    derivative = (error - previous_error)/dt;
    output = (Kp*error + Ki*integral + Kd*derivative);
    previous_error = error;
    phi = round(output);
%     if abs(error) > .1
%         phi = round(output);        
%     end
    
    
    
    %easy
    %added delay so that the pid has time to adjust
%     if(corrected_flag)
%         corrected_flag = 0;
%     else
%         if(abs(error) > .15)
%             if(error < 0)
%                 phi = phi - 1;
%             else
%                 phi = phi + 1;
%             end
%         else
%             phi = phi;
%         end
%         corrected_flag = 1;
%     end

       
    phis(inc) = phi;
    errors(inc) = error;
    
   
% -----------------------MULTIPLYING BY SIN AND COS------------------------
    %%set new phi to waves 

    
    sinArray = input(inc)*sinWave(inc+phi);
    cosArray = input(inc)*cosWave(inc+phi);    
    
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
% Summing previous 100 (Tc/Ts) values        
        for inc2 = inc-(INTEGRAL_ITERATIONS-1):inc
            y1(inc) = y1(inc) + s1(inc2);
            y2(inc) = y2(inc) + s2(inc2);
        end
        y1(inc) = y1(inc) / INTEGRAL_ITERATIONS;
        y2(inc) = y2(inc) / INTEGRAL_ITERATIONS;
    end 
    bit1(inc) = sign(y1(inc));
end

% -----------------------XOR With Gold Codes-------------------------------





% -----------------------PLOTS------------------------------------
subplot(2,2,1)
%figure(1)
stem(input_clean(1701:1850));
title('Ideal Input');
xlabel('time in sample intervals');
ylabel('Amplitude');
subplot(2,2,2);
%figure(2)
stem(input(1701:1850));
title('The input to our receiver');
xlabel('time in sample intervals');
ylabel('Amplitude');
subplot(2,2,3);
%figure(3)
stem(y1(1701:1850));
hold on;
plot(bit1(1701:1850));
hold off;
title('Output Waveform');
xlabel('time in sample intervals');
ylabel('Amplitude');
subplot(2,2,4);
plot(phis(1:length(phis)-((20*SAMPLE_RATE_MULT))))
hold on;
plot(errors(1:length(phis)-((20*SAMPLE_RATE_MULT))));
hold off;
title('Phase of the System');
xlabel('time in sample intervals');
ylabel('phi');

% print out the final phase
final_phi = phis(length(phis)-((20*SAMPLE_RATE_MULT))) % * pi/5







 
 



