clear;

load co2data

Input_exp1 = Input_exp1-...
   ones(size(Input_exp1,1),1)*mean(Input_exp1(1:50,:));
Output_exp1 = Output_exp1-...
   mean(Output_exp1(1:50,:));
Input_exp2 = Input_exp2-...
   ones(size(Input_exp2,1),1)*mean(Input_exp2(1:50,:));
Output_exp2 = Output_exp2-...
   mean(Output_exp2(1:50,:));

Ts = 0.5; % Sample time is 0.5 min
ze = iddata(Output_exp1,Input_exp1,Ts);
zv = iddata(Output_exp2,Input_exp2,Ts);

% Set time units to minutes
ze.TimeUnit = 'min';
% Set names of input channels
ze.InputName = {'ConsumptionRate','Current'};
% Set units for input variables
ze.InputUnit = {'kg/min','A'};
% Set name of output channel
ze.OutputName = 'Production';
% Set unit of output channel
ze.OutputUnit = 'mg/min';

% Set validation data properties
zv.TimeUnit = 'min';
zv.InputName = {'ConsumptionRate','Current'};
zv.InputUnit = {'kg/min','A'};
zv.OutputName = 'Production';
zv.OutputUnit = 'mg/min';

figure   % Open a new MATLAB Figure window
plot(zv) % Plot the validation data

Ze1 = ze(1:1000);
Zv1 = zv(1:1000);

Ge = spa(ze);
bode(Ge);

% model estimation
Mimp = impulseest(Ze1,60);

% step response
step(Mimp)

% model estimation
Mimp = impulseest(Ze1,60);




