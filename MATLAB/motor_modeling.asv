t = 0:0.05:10;
R = 2.0;
torque = 0;
free_run_speed = 24.46; % rad/s
stall_torque = 1.2; % Nm
stall_i = 5; % A
stall_i_voltage = 12; % V
ke = stall_i_voltage / free_run_speed
kt = stall_torque / stall_i

v = 12;
w = (v - R * torque / kt) / ke

