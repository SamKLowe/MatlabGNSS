%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%Trunnel GNSS Reciever Dicrete Costas Loop Simulation
%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;

% look up LFSR for code generation

%%% Adjustment Constants
SAMPLE_FREQUENCY = 10230000;
SAMPLE_RATE_MULT = 2; % default rate is 5 samples per period


%%%%%Open input file snr3/snr100
filename='snr3.txt';

%Signal Generation Constants
SIG2NOISE_RATIO = 7; % decrease to add more noise
INTEGRAL_ITERATIONS = 10; %increase to increase the effect of the low pass filter
PHI_ADJUST = 0; %must do some extra stuff to it to turn it into a cos wave
PHI_INIT = ((0-PHI_ADJUST)-1)-5;   % this number * pi/2 some weird math to line up right
START_PHI = (pi/(5*SAMPLE_RATE_MULT)) * PHI_INIT; %used to initialize 
STREAM_SAMPLES = 20100; %will eventually auto detect this
SAMPLES = 2000; % amount of samples to use
EXTRA_SAMPLES = (5*SAMPLE_RATE_MULT) - (5+PHI_INIT); %used for phase offset

%C/A arrays to use for various samples
%in future can add a function that builds this
matchme=[  1 1 1 1 1 1 1 1 1 1 ];
cw = [0 1]; %20
%cw = [1 0 1 0 1 0 1 1 0 0]; %100
%cw = [1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 1 1 1 1 1 1 1 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0 1 0 1 0 1 0 1 0 1 1 0 0 1 0]; % 500
rand_bit = 0;
goldcode1 = [ -1 1 -1 1 -1 1 -1 1 1 -1 -1 1 -1 1 1 1 1 1 1 1 ]
goldcode2=[ -1 -1 -1 1 -1 -1 1 -1 -1 -1 1 1 -1 1 -1 -1 -1 1 -1 1];


for l = (1 : 500)
    rand_bit = randi(2,1,1);
    rand_bit = rand_bit - 1;
    cw = [cw rand_bit];
end
cw = [cw goldcode2];
for l = (1 : 500)
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




y = conv(matchme, cwSamp);
test1 = autocorr(goldcode1);
test2 = autocorr(goldcode2);

subplot(2,1,1)
plot(test2);
title('Old Gold Code Autocorrelation');
xlabel('Shifts');
ylabel('Correlation');
subplot(2,1,2)
plot(test1);
title('New Gold Code Autocorrelation');
xlabel('Shifts');
ylabel('Correlation');