function dydt = quadcopter_ode(t,y,u)

[H,Ts,drone1_info, drone2_info] = drones_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;

%% Unpack the state and input vectors


%states drone 1
position_x_d1= y(1);
position_z_d1= y(2);
pitch_d1= y(3);
velocity_x_d1= y(4);
velocity_z_d1= y(5);
velocity_pitch_d1= y(6);

%states drone 2
position_x_d2= y(7);
position_z_d2= y(8);
pitch_d2= y(9);
velocity_x_d2= y(10);
velocity_z_d2= y(11);
velocity_pitch_d2= y(12);

%control variables drone 1
diff_mode_d1  = u(1,:);
common_mode_d1 = u(2,:);

%control variables drone 2
diff_mode_d2  = u(3,:);
common_mode_d2 = u(4,:);

%%Equations of motion
x_acceleration_d1 = -(1/mass)*sin(pitch_d1)* common_mode_d1;
z_acceleration_d1 = -gravitational_acceleration +(1/mass)*cos(pitch_d1)* common_mode_d1;
pitch_acceleration_d1 = (arm_moment/inertia_moment)*diff_mode_d1;

x_acceleration_d2 = -(1/mass)*sin(pitch_d2)* common_mode_d2;
z_acceleration_d2 = -gravitational_acceleration +(1/mass)*cos(pitch_d2)* common_mode_d2;
pitch_acceleration_d2 = (arm_moment/inertia_moment)*diff_mode_d2;
%%Equations of thrust forces
%throttle_ref = alpha*[diff_mode common_mode]';
%f_velocity = (-[f1 f2]'/tau) + throttle_ref/tau;

%% 

drone1= [velocity_x_d1;velocity_z_d1;velocity_pitch_d1;x_acceleration_d1;z_acceleration_d1;pitch_acceleration_d1];
drone2 = [velocity_x_d2;velocity_z_d2;velocity_pitch_d2;x_acceleration_d2;z_acceleration_d2;pitch_acceleration_d2];

dydt=[drone1; drone2];

end