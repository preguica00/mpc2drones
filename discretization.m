function [c, ceq] = discretization(y,init_drone1,init_drone2)

[H,Ts,drone1_info, drone2_info] = drones_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;



%% Unpacking drones
% drone 1 
id1_u1 = drone1_info(1);
id1_u2 = drone1_info(2);
id1_x = drone1_info(3);
id1_z = drone1_info(4);
id1_theta = drone1_info(5);
id1_dotx = drone1_info(6);
id1_dotz = drone1_info(7);
id1_dottheta = drone1_info(8);

% drone 2
id2_u1 = drone2_info(9);
id2_u2 = drone2_info(10);
id2_x = drone2_info(11);
id2_z = drone2_info(12);
id2_theta = drone2_info(13);
id2_dotx = drone2_info(14);
id2_dotz = drone2_info(15);
id2_dottheta = drone2_info(16);


%% Assigning drone variables
% drone 1
mode_diff_d1 = y(id1_u1);
mode_common_d1 = y(id1_u2);
x_d1 = y(id1_x);
z_d1 = y(id1_z);
theta_d1 = y(id1_theta);
x_velocity_d1 = y(id1_dotx);
z_velocity_d1 = y(id1_dotz);
angular_velocity_d1 = y(id1_dottheta);

% drone 2

mode_diff_d2 = y(id2_u1);
mode_common_d2 = y(id2_u2);
x_d2 = y(id2_x);
z_d2 = y(id2_z);
theta_d2 = y(id2_theta);
x_velocity_d2 = y(id2_dotx);
z_velocity_d2 = y(id2_dotz);
angular_velocity_d2 = y(id2_dottheta);

%%
current_state=[init_drone1;init_drone2];
% state =[current_state; [x_d1;z_d1;theta_d1;x_velocity_d1;z_velocity_d1;angular_velocity_d1;x_d2;z_d2;theta_d2;x_velocity_d2;z_velocity_d2;angular_velocity_d2]];
state =[current_state, [x_d1,z_d1,theta_d1,x_velocity_d1,z_velocity_d1,angular_velocity_d1,x_d2,z_d2,theta_d2,x_velocity_d2,z_velocity_d2,angular_velocity_d2]'];
control=[mode_diff_d1;mode_common_d1; mode_diff_d2;mode_common_d2];


% Run discrete prediction
ceq = [];
% for j=1:2
for i = 1:H
%     ceq = [ceq; [(state(j+1,i+1) - timestep_euler(Ts,state(j,i), control(j,i)))]];
    ceq = [ceq; [(state(:,i+1) - timestep_euler(Ts,state(:,i), control(:,i)))]];

end


c=[];
end
