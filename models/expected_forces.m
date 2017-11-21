clear;

% From Petzel Fall Factor Data
% https://www.petzl.com/US/en/Sport/Forces-at-work-in-a-real-fall

m_climber = 80; % kg
m_belayer = 80; % kg 
g = 9.81; % m/s/s


%%  0.3 Fall Factor %%

% Falll with 2 m free fall with 7 m rope in system. 

F_climber_rope = 2500; % N
F_belayer_rope = 1500; % N

h = 2; % m free fall height

% Force on climber
F_climb_net = F_climber_rope - m_climber * g;
% Upwards climber acceleration
a_climb = F_climb_net / m_climber % m/s/s

% Force on belayer
F_belayer_net = F_belayer_rope - m_belayer * g; 
% Updards belayer max acceleration
a_belay = F_belayer_net / m_belayer 

%% 0.7 Fall Factor

% Falll with 2 m free fall with 3 m rope in system. 


F_climber_rope = 3000; % N
F_belayer_rope = 2000; % N

h = 2; % m free fall height

F_climb_net = F_climber_rope - m_climber * g;
a_climb = F_climb_net / m_climber % m/s/s

F_belayer_net = F_belayer_rope - m_belayer * g; 
a_belay = F_belayer_net / m_belayer

%% Worser Case concieveable Falls

% Consider a worst worst case scenario
% Scale Petzel forces with max climber weight. Use lighter belayer

% Max weight climber
m_climber_max = 150; % kg These two people shouldnt be partners
m_belayer_min = 60;  % kg


% 0.3 FF
F_climber_rope = 2500; % N
F_belayer_rope = 1500; % N

F_climber_rope = F_climber_rope * m_climber_max / m_climber;
F_belayer_rope = F_belayer_rope * m_climber_max / m_belayer;

F_belayer_net = F_belayer_rope - m_belayer_min * g 
a_belay = F_belayer_net / m_belayer_min


% 0.7 FF 
F_climber_rope = 3000; % N
F_belayer_rope = 2000; % N

F_climber_rope = F_climber_rope * m_climber_max / m_climber;
F_belayer_rope = F_belayer_rope * m_climber_max / m_climber;

F_belayer_net = F_belayer_rope - m_belayer_min * g 
a_belay = F_belayer_net / m_belayer_min % ZOOM! 

% So 3.16 kN Is a reasonable upper bound on maximum brake force conceieveable. 



