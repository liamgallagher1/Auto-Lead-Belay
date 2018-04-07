clear;

% using lm for this test
M = csvread('logs/upper_pulley_id_3_93_1_22.csv', 1, 0);

sm_raw_amp = M(:, 8:9);
sm_current = M(:, 12); % Noisy estimate of the current
sm_count   = M(:, 14); % Encoder counts
sm_v = 12 * M(:, 19); % signed, in volts
Ts = 0.001;

input = sm_v;
output = [sm_current, sm_count];

ze = iddata(output, input,Ts);
% Set time units to minutes
ze.TimeUnit = 'seconds';
% Set names of input channels
ze.InputName = {'Voltage'};
% Set units for input variables
ze.InputUnit = {'V'};
% Set name of output channel
ze.OutputName = {'Current','Encoder Counts'};
% Set unit of output channel
ze.OutputUnit = {'A','tics'};

figure   % Open a new MATLAB Figure window
plot(ze) % Plot the validation data

% Make bode plot
Ge = spa(ze);
bode(Ge);

% model estimation
Mimp = impulseest(ze,60);

% step response
step(Mimp);


z_curr = iddata(sm_current, input,Ts);
z_count = iddata(sm_count, input,Ts);

d_curr = delayest(z_curr);
d_count = delayest(z_curr);

t_delay = Ts * d_count;

%% Do CV here
%NN1 = struc(2:5,1:5,5);
%selstruc(arxstruc(ze(:,:,1),zv(:,:,1),NN1))


%% Estimate a ARX model
Opt = tfestOptions('Display','on');
np = 2;
ioDelay = t_delay;

tf_curr = tfest(z_curr, np, [], ioDelay, Opt);

tf_count = tfest(z_count, np, [], ioDelay, Opt);

compare(z_curr, tf_curr);

compare(z_count, tf_count);

resid(z_count , tf_count)

resid(z_curr , tf_curr)

%% SS Estimation
mCanonical = n4sid(ze, 3,'SSParameterization','canonical','InputDelay', d_count);
present(mCanonical);  %  Display model properties

opt = n4sidOptions('Focus','simulation','Display','on');
mn4sid = n4sid(ze,1:8,'InputDelay', d_count, opt);
















