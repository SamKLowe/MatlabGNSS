% real demo

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%Trunnel GNSS Reciever Dicrete Costas Loop Simulation
%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;

%%% Adjustment Constants
SAMPLE_FREQUENCY = 1023000;
SAMPLE_RATE_MULT = .8; % default rate is 5 samples per period

%Signal Generation Constants
INTEGRAL_ITERATIONS = 3; %increase to increase the effect of the low pass filter
PHI_ADJUST = 3; %must do some extra stuff to it to turn it into a cos wave
PHI_INIT = ((0-PHI_ADJUST)-1)-5;   % this number * pi/2 some weird math to line up right
START_PHI = (pi/(5*SAMPLE_RATE_MULT)) * PHI_INIT; %used to initialize 
STREAM_SAMPLES = 16384; %will eventually auto detect this
SAMPLES = 2000; % amount of samples to use
EXTRA_SAMPLES = (5*SAMPLE_RATE_MULT) - (5+PHI_INIT); %used for phase offset
chips = STREAM_SAMPLES/(5*SAMPLE_RATE_MULT);


%Early Late detector variables and preload with zeros
ELearlyShift = CQueue();
ELpromptShift = CQueue();
ELlateShift = CQueue();

ELearlyShiftReg = zeros(4096);
ELpromptShiftReg = zeros(4096); 
ELlateShiftReg = zeros(4096);



E_L_INTEGRAL_ITERATIONS = 4096;

ELearlyPow = 0;
ELpromptPow = 0;
ELlatePow = 0;

ELsEarly = 0;
ELsPrompt = 0;
ELsLate = 0;

ELearlyOffset = -2;
ELpromptOffset = 0;
ELlateOffset = 2;

ELerror = 0;

%%%%%Open input file snr3/snr100
filename='GPS_test120s_5_4M.dat';



%%%%% Unsorted constants
costasCarrier = 0;
inc = 0;
y1 = 0;
y2 = 0;

s1 = zeros(INTEGRAL_ITERATIONS);
s2 = zeros(INTEGRAL_ITERATIONS);

sampleInc = 1;

fileID= fopen(filename, 'r');%open file

%%build costas cosine and sine waves

sinMult = dsp.SineWave();
sinMult.Frequency = SAMPLE_FREQUENCY;
sinMult.Amplitude = 1;
sinMult.PhaseOffset = 0;
sinMult.SamplesPerFrame =  500;
sinMult.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*5*2;
sinMult.OutputDataType = 'single';

cosMult = dsp.SineWave();
cosMult.Frequency = SAMPLE_FREQUENCY;
cosMult.Amplitude = 1;
cosMult.PhaseOffset = pi/2;
cosMult.SamplesPerFrame = 500;
cosMult.SampleRate = SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*5*2;
cosMult.OutputDataType = 'single';

sinWave = step(sinMult);
cosWave = step(cosMult);



corrected_flag = 0;

%Costas Integrator variables
y1 = 0;
y2 = 0;

Costs1 = CQueue();
Costs2 = CQueue();

%%%% PID Values

setpoint = 0;
error = 0;
previous_error = 0;
integral = 0;
derivative = 0;
output = 0;
Ki = 4000000;   % divided by 1.024 Million (dt)
Kd = .0000000001; % multiplied by 1.024 Million (dt)
Kp = .000000000002;
% Kp = 2;
dt = 1/(SAMPLE_FREQUENCY*SAMPLE_RATE_MULT*5);

%phase variable
phi = 0;

%xor variables
shift=[];
shiftindex=0;
curr_samp=0;
n=0;
xoredbits=[];
lengthofgoldcode=1023;
next_curr_samp=0;
xoredgraph=[];
early_bit = 0;
prompt_bit = 0;
late_bit = 0;
xoredmag=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate C/A code to use
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%currently just using one of the sattelites we find
goldcode = cacode(22, 4);
goldcode = goldcode';

%change zeros to (-1)
for i = 1:length(goldcode)
    if(goldcode(i) == 0)
        goldcode(i) = -1;
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the range is weird because we need to make room for the phase offset
% for inc = (ceil(SAMPLES / 10):SAMPLES - ceil(SAMPLES / 10))
% for inc = (20*SAMPLE_RATE_MULT:SAMPLES - (20*SAMPLE_RATE_MULT)) 

% Scan in early samples
x =  double(fread(fileID,[2,30],'*uint32'));



for inc = (30:STREAM_SAMPLES*SAMPLE_RATE_MULT*5)
    
   %Read in data from ADC comment out if using code generated input
   input_pair =  double(fread(fileID,[2,1],'*uint32'));
   input = (input_pair(1,1)/1.0e9-2.1)';
   inc = inc
   
    % create an always positive Phi
    if(phi < 0)
        phiPos = 10 - (mod(abs(phi), 10));
    else
        phiPos = phi;
    end
   
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%COSTAS LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   
% -----------------------MULTIPLY INPUT W/ C/A CODE------------------------   

    % PLL will just lock onto our message signal
    costas_input = input*goldcode(mod(inc+phiPos, (lengthofgoldcode * 5*SAMPLE_RATE_MULT))+1);
    
   
% -----------------------MULTIPLYING BY SIN AND COS------------------------
    % set new phi to waves 
    % not a great way to do it computing wise but I could not get matlab to accept single multiplication

    % generate carrier wave to send up to early late detector to decode
    % BPSK
    
    costasCarrier = sinWave(mod(inc+phiPos, 20)+1);
    
    sinArray = costas_input*costasCarrier;
    cosArray = costas_input*cosWave(mod(inc+phiPos, 20)+1);
        
    
    sinElement = sinArray;
    cosElement = cosArray;
    
    Costs1.push(sinElement);
    Costs2.push(cosElement);
    
    
    y1 = 0;
    y2 = 0;

% -----------------------LOW PASS FILTER-----------------------------------
    s1 = Costs1.content();
    s2 = Costs2.content();

    if inc-30 < INTEGRAL_ITERATIONS
     %If sample index is less than 100 (Tc/Ts) then we sum available previous
     %samples divided by the number of samples
        for inc2=1:inc-29
            y1 = y1 + s1{inc2};
            y2 = y2 + s2{inc2};
        end
        y1 = 0;
        y2 = 0;
    else
    % Summing recent previous 100 (Tc/Ts) values        
        for inc2 = 1:INTEGRAL_ITERATIONS
            y1 = y1 + s1{inc2};
            y2 = y2 + s2{inc2};
        end
        y1 = y1 / INTEGRAL_ITERATIONS;
        y2 = y2 / INTEGRAL_ITERATIONS;
        
        Costs1.pop();
        Costs2.pop();
    end 
    
    
    
    
% -----------------------LOOP FILTER (CONTROL LOOP)-----------------------
    %PID controller

    pidY1 = y1;    
    pidY2 = y2;
    error = pidY1*pidY2*2; % Error Multiplication
    
    integral = integral + (error * dt);
    derivative = (error - previous_error)/dt;
    output = (Kp*error + Ki*integral + 0*derivative);
    previous_error = error;
    phi = round(output); 
    
    

    
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%EARLY LATE DETECTOR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-----------------------bpsk decode----------------------------------
    
    %multiply incoming signal with phase matched sin wave
    ELinput = input * costasCarrier;
    
% -----------------------PRN multiplication-------------------------------
    %will decode the CDMA as we go so our integrator can produce a bit
    
    ELsEarly = ELinput*goldcode(mod(inc+ELearlyOffset, lengthofgoldcode * 5*SAMPLE_RATE_MULT)+1);
    ELsPrompt = ELinput*goldcode(mod(inc+ELpromptOffset, lengthofgoldcode * 5*SAMPLE_RATE_MULT)+1);
    ELsLate  = ELinput*goldcode(mod(inc+ELlateOffset, lengthofgoldcode * 5*SAMPLE_RATE_MULT)+1);
    
 % -----------------------Integrate and Dump-------------------------------   
    y = ELsEarly;
    ELearlyShift.push(ELsEarly);
    ELpromptShift.push(ELsPrompt);
    ELlateShift.push(ELsLate); 
    
    ELearlyPow = 0;
    ELpromptPow = 0;
    ELlatePow = 0;
 
    % load new value into queues
    ELearlyShiftReg = ELearlyShift.content();
    ELpromptShiftReg = ELpromptShift.content();
    ELlateShiftReg = ELlateShift.content();
    
    if inc<=E_L_INTEGRAL_ITERATIONS + 30
      
            for inc2 = 1:inc-29            
                
                
                ELearlyPow = ELearlyPow + (ELearlyShiftReg{inc2}*.001);
                ELpromptPow = ELpromptPow + (ELpromptShiftReg{inc2}*.001);
                ELlatePow = ELlatePow + (ELlateShiftReg{inc2}*.001);
                %add Q
            end
        
    
    else       
    
        
    % Summing recent previous 100 (Tc/Ts) values        
        for inc2 = 1:E_L_INTEGRAL_ITERATIONS
            ELearlyPow = (ELearlyPow + ELearlyShiftReg{inc2}*.0001);
            ELpromptPow = (ELpromptPow + ELpromptShiftReg{inc2}*.0001);
            ELlatePow = (ELlatePow + ELlateShiftReg{inc2}* .0001);
            %add Q
        end
        
        ELearlyShift.pop();
        ELpromptShift.pop();
        ELlateShift.pop();
     
    end  
    
    % discriminator
    
    ELerror = ELearlyPow - ELlatePow;
    
    % E/L PID
    
    
    
    % graphing variables. keep four to 
    
    if(mod(inc,10))
        graph1(sampleInc) = ELearlyPow;
        graph2(sampleInc) = ELpromptPow;
        graph3(sampleInc) = ELlatePow;
        graph4(sampleInc) = 0;
        sampleInc = sampleInc +1;
        
    end
    
    
end



    

%%%%%%%%%%%%%%%%%%%%%PLOTS
subplot(2,2,1)
figure(1)
%stem(goldcode(1701:1850));
stem(graph1);
plot(xoredgraph);
title('Xor Magnitude');
xlabel('Chip Number');
ylabel('Magnitude');
subplot(2,2,2);
%figure(2)
stem(graph2);
title('Input');
xlabel('time in sample intervals');
ylabel('Amplitude');
subplot(2,2,3);
%figure(3)
%stem(y1(1701:1850));
stem(graph3);
title('Costas Output Waveform');
xlabel('time in sample intervals');
ylabel('Amplitude');
subplot(2,2,4);
plot(graph1)
hold on;
plot(graph2)
hold on;
plot(graph3)
hold off;
title('Phase/Error of the System');
xlabel('time in sample intervals');
ylabel('phi');

% print out the final phase
final_phi = phi % * pi/5
final_early = ELearlyPow
final_prompt = ELpromptPow
final_late = ELlatePow

fclose(fileID);

