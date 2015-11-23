
% ------------------------------------------------------------------------
% --------------------------GENERATOR PART--------------------------------
% Lets generate a sample input signal with known phase offset or you can
% use your own signal  
%fc = 4000;                   % Carrier Frequency example
fc=1023000;%1575420;
fs=10*fc;
 t = 0:800000;                 % time scale

 %fs = 4e5;                    % Sampling Frequency example
 m = cos(2*pi*150*t/fs);      % sample message signal
 c = cos(2*pi*fc*t/fs + (pi/3)); % carrier signal with phase offset
 n = 1*randn(1,length(m));      % random noise generation
 st = m.*c+(n/50); 
 % DSB-SC signal
% -----------------------------------------------------------------------
figure;
plot(st);title('Input Signal');

% ---------------------------RECEIVER PART-------------------------------
N = length(st);              
t = 0:1:N-1;                 % Time vector
 phi=(0);
s1=(0);
s2=(0);
y1=(0);
y2=(0);
output = (0);
for q=1:N
phi(q) = 0;            % Phase vector of VCO initialize
 s1(q) = 0; 
 s2(q) = 0;
 y1(q) = 0;
y2(q) = 0;
output(q) = 0;
 end
for i = 1:N
    
    if i>1
% The step in which phase is changed is pi*5*10*-5, it can be varied.        
       
        phi(i) = phi(i-1) - (2*10^-5)*pi*sign(y1(i-1)*y2(i-1));
        %shifting a square pulse over the signal to set everything to the
        %rails of positive or negative 1.
    end
    
    s1(i) = st(i) * cos(2*pi*fc*t(i)/fs  + phi(i));
    s2(i) = st(i) * sin(2*pi*fc*t(i)/fs  + phi(i));
    %multiplies the signal by sin and cos like in the block diagram

% -----------------------INTEGRATOR------------------------------------
    if i<=100
%  If sample index is less than 100 (Tc/Ts) then we sum available previous
%  samples
        for j=1:i
            y1(i) = y1(i) + s1(j);
            y2(i) = y2(i) + s2(j);
        end
      
    else
% Summing previous 100 (Tc/Ts) values        
        for j = i-99:i
            y1(i) = y1(i) + s1(j);
            y2(i) = y2(i) + s2(j);
        end
    end
%----------------------------------------------------------------------    
end

output = sign(s1);

figure;
plot(t,output, t, s1);title('Binary Interpretation');
xlabel('Time');ylabel('Amplitude');

figure;
plot(t,y1);title('Output signal');
xlabel('Time');ylabel('Amplitude');
figure;
plot(t,phi);title('Phase for Signal vs Time');
xlabel('Time');ylabel('Phase');

%For getting initial phase of carrier wave we approximate it with final
%value of phase attained by our VCO

phase = phi(end)

