clear;
% Smoothes data using a gaussian process

filename = '../../../pi/motor_driver/logs/timestamp_test53_5_46.csv';

% FUCK THIS
%A = csvread(filename, 3, 0, [3, 0, 3, 1]);
% num_stamps = 242603; %A(1);
% num_steps = A(2);
A = csvread(filename, 4, 0);

time_stamps_us = A(1:length(A) / 2);
time_stamps_s = time_stamps_us / 10^6;

counts = A(length(A)/2 + 1:end);

sub_segment = 2000;
time_stamps_s = time_stamps_s(2*sub_segment+1:3*sub_segment);
counts = counts(2*sub_segment+1:3*sub_segment);


% lets fit a GP

% gpm = fitrgp(time_stamps_s, counts, ...
%     'FitMethod', 'fic', ...
%     'Verbose',1,...\\\\\\
%     'SigmaLowerBound',0.02,...
%     'ComputationMethod','v',...
%     'KernelFunction','squaredexponential',...
%     'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',...
%     struct('AcquisitionFunctionName', 'expected-improvement-per-second-plus'));
% 
% 
% 
% x = time_stamps_s;
% y = counts;
% ypred = resubPredict(gpm);
% 
% plot(x,y,'r.');
% hold on
% plot(x,ypred,'k'); %'LineWidth',2);
% xlabel('x');
% ylabel('y');
% title('Impact of Optimization');
% hold off
