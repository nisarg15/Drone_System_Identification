clear all
close all
clc


load sysid_data

bodyvel_time = mavroslocalpositionvelocitybody(:,3)+(mavroslocalpositionvelocitybody(:,4)/(10^9));
bodyvel = mavroslocalpositionvelocitybody(:,6:8);

inputs_time = mavrosrcin(:,3)+(mavrosrcin(:,4)/(10^9));
inputs = mavrosrcin(:,7:10);

imudata_time = mavrosimudata(:,3)+(mavrosimudata(:,4)/(10^9));
imudata_q = mavrosimudata(:,6:9);

imudata_omega = mavrosimudata(:, 19:21);

imudata_lpp_time = mavroslocalpositionpose(:,3)+(mavroslocalpositionpose(:,4)/(10^9));
imudata_lpp_trans = mavroslocalpositionpose(:,6:8)-mavroslocalpositionpose(1,6:8);
imudata_lpp_orientation = mavroslocalpositionpose(:,9:12);

vicondata_time = viconm500joecm500joec(:,3)+(viconm500joecm500joec(:,4)/(10^9));
vicondata_q = viconm500joecm500joec(:,10:13);
vicondata_trans = viconm500joecm500joec(:,7:9)-viconm500joecm500joec(1,7:9);

min_time = min([min(imudata_time), min(vicondata_time) min(imudata_lpp_time) min(bodyvel_time) min(inputs_time)]);

imudata_time = imudata_time-min_time;
vicondata_time = vicondata_time-min_time;
imudata_lpp_time = imudata_lpp_time-min_time;
bodyvel_time = bodyvel_time-min_time;
inputs_time = inputs_time-min_time;

% 1: Throttle
% 2: Roll
% 3: Pitch
% 4: Yaw
inputs_resamp = interp1(inputs_time, inputs, vicondata_time, 'linear', 'extrap');

% 1 q_w, 2 q_x, 3 q_y, 4 q_z
imudata_q_resamp = interp1(imudata_time, imudata_q, vicondata_time, 'linear', 'extrap');
% 1 wx (roll), 2 wy (pitch), 3 wz (yaw)
imudata_omega_resamp = interp1(imudata_time, imudata_omega, vicondata_time, 'linear', 'extrap');
% 1 x, 2 y, 3 z
imudata_lpptrans_resamp = interp1(imudata_lpp_time, imudata_lpp_trans, vicondata_time, 'linear', 'extrap');
imudata_lpp_orientation_resamp = interp1(imudata_lpp_time, imudata_lpp_orientation, vicondata_time, 'linear', 'extrap');
bodyvel_resamp = interp1(bodyvel_time, bodyvel, vicondata_time, 'linear', 'extrap');

qeul = quaternion(deg2rad([-90,0,0]), 'euler', 'ZYX', 'frame');

for i=1:length(vicondata_q)
    quat_v_v = quaternion([vicondata_q(i,4), vicondata_q(i,1:3)]);
    quat_v_m = quat_v_v;
    quat_i = quaternion([imudata_q_resamp(i,4), imudata_q_resamp(i,1:3)]);
    quatest(i) = (quat_i)*-qeul;
    quatest(i) = normalize(quatest(i));
    A1 = eulerd(quat_v_m, 'ZYX', 'frame');
    A2 = eulerd(quatest(i), 'ZYX', 'frame');
    
    euler_v(i,:) = A1;
    euler_i(i,:) = A2;
    [A,B,C,D] = parts(quat_v_m);
    quat_v_m_mat(i,:) = [A,B,C,D];

end

rollang = euler_i(:,2);
rollin = inputs_resamp(:,2);
sideslip = bodyvel_resamp(:,2);
rollomega = imudata_omega_resamp(:,1);
lateral_position = vicondata_trans(:,1);

% 0.01 is the time step; 100 Hz

% Roll input to Roll Angle
figure(1)
subplot(2,1,1)
plot(vicondata_time, rollin);
xlabel('time (seconds)');
ylabel('amplitude (pwm counts)')
subplot(2,1,2)
plot(vicondata_time, rollang);
xlabel('time (seconds)');
ylabel('angle (degrees)')
sgtitle('Roll Input to Roll Angle');


% Roll angle to Lateral Velocity
figure(2)
subplot(2,1,1)
plot(vicondata_time, rollang);
xlabel('time (seconds)');
ylabel('angle (degrees)')
subplot(2,1,2)
plot(vicondata_time, sideslip);
xlabel('time (seconds)');
ylabel('lateral velocity (m/s)')
sgtitle('Roll Angle to Lateral Velocity');

% Lateral Velocity to Lateral Position
figure(3)
subplot(2,1,1)
plot(vicondata_time, sideslip);
xlabel('time (seconds)');
ylabel('lateral velocity (m/s)')
subplot(2,1,2)
plot(vicondata_time, lateral_position);
xlabel('time (seconds)');
ylabel('lateral position (m)')
sgtitle('Lateral Velocity to Lateral Position');
