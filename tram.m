%% System Parameters
v_line  = 600;   % V

w_b     = 314;   % rad/s - motor rated speed
eta     = 0.9;   % / - efficiency (neglecting excitation and iron losses)
tau_a   = 10e-3; % s - Armature time constant
tau_e   = 1;     % s - Excitation time constant
ve_max  = 120;   % V - Rated excitation voltage
ie_max  = 1;     % A - Rated excitation current

m_train = 10e3;                % kg - Train mass
m_pass  = 80;                  % kg - passenger mass 
n_pass  = 200;                 % / - Number of passengers
m_load  = m_pass*n_pass;       % kg - total mass of passengers
m_t     = m_train + m_load;    % kg - total mass of train and passengers

s_b     = 60;        % m/s - base speed

%% Calculated Parameters
Ks      = v_line*eta/(ie_max*w_b);      % Nm/A^2 or Vs/rad*A - constant for EMF and Torque expression
r_whl   = s_b/w_b*10/36;                % m - Tram wheel radius

acc     = s_b/25*10/36;                 % m/s^2 - Train acceleration
F_tr    = m_t*acc;                      % N - Traction force
Te_n    = 4/3*F_tr*r_whl;               % Nm - Torque at rated speed (Pe = P_trac + P_fr) )

% ia_n    = Te_n/(Ks*ie_max);                         % A- rated armature current
ia_n    = 575;                         % A- rated armature current
e_n     = Ks*ie_max*w_b;               % V - max back EMF

Re      = ve_max/ie_max;               % Ohm - Resistance of excitation circuit 
Le      = tau_e*Re;                    % H - Inductance of excitation ciruit

Ra      = (v_line-e_n)/ia_n;         % Ohm - Resistance of armature circuit 
La      = tau_a*Ra;                    % H - Inductance of armature circuit

J       = m_t*r_whl^2;
B       = 1/3*Te_n*1/w_b;

g       = 9.81;                        % m/s^2 - acceleration due to gravity

%% Transfer Functions
Ga = tf(1,[La Ra]);  % Armature Transfer Function
Ge = tf(1, [Le Re]); % Excitation Transfer Function
Gm = tf(1, [J B]);   % Mechanical Load Transfer function

%% Bandwidths
T_s   = 1;         % Required settling time for Speed
tau_s = 1/5*T_s;
ws    = 1/tau_s;    % Bandwidth of Speed Controller
wi_a  = 10*ws;      % Bandwidth of Armature Current Controller
wi_e  = 5/tau_e;    % Bandwidth of Excitation Current Controller

%% Anti-windup for PI blocks with back-calculation
kb_a   = 1/tau_a;
kb_e   = 1/tau_e;
kb_spd = 1/tau_s;

%% Controller Tuning with PIDTuner

% Armature Current Controller
phase_m_a = 90; 
opt_a = pidtuneOptions("PhaseMargin", phase_m_a);
Ci_a = pidtune(Ga,"pi", wi_a, opt_a);
Kpa = Ci_a.Kp;
Kia = Ci_a.Ki;

% Excitation Current Controller
phase_m_e = 80; 
opt_e = pidtuneOptions("PhaseMargin", phase_m_e);
Ci_e = pidtune(Ge,"pi", wi_e, opt_e);
Kpe = Ci_e.Kp;
Kie = Ci_e.Ki;

% Speed Controller
phase_m_s = 60; 
opt_s = pidtuneOptions("PhaseMargin", phase_m_s);
Cm  = pidtune(Gm,"pi", ws, opt_s);
Kpm = Cm.Kp;
Kim = Cm.Ki;

sim("tram_sim")