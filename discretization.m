function [c, ceq] = discretization(y,x_init_d1,z_init_d1,theta_init_d1,xvelocity_init_d1,zvelocity_init_d1,angvelocity_init_d1,x_init_d2,z_init_d2,theta_init_d2,xvelocity_init_d2,zvelocity_init_d2,angvelocity_init_d2)

[H,Ts,id1_u1,id1_u2,id1_x,id1_z,id1_theta,id1_dotx,id1_dotz,id1_dottheta, id2_u1,id2_u2,id2_x,id2_z,id2_theta,id2_dotx,id2_dotz,id2_dottheta] = drones_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;

% Unpacking drone 1 

mode_diff_d1 = y(id1_u1);
mode_common_d1 = y(id1_u2);
x_d1 = y(id1_x);
z_d1 = y(id1_z);
theta_d1 = y(id1_theta);
x_velocity_d1 = y(id1_dotx);
z_velocity_d1 = y(id1_dotz);
angular_velocity_d1 = y(id1_dottheta);

% Unpacking drone 2 

mode_diff_d2 = y(id2_u1);
mode_common_d2 = y(id2_u2);
x_d2 = y(id2_x);
z_d2 = y(id2_z);
theta_d2 = y(id2_theta);
x_velocity_d2 = y(id2_dotx);
z_velocity_d2 = y(id2_dotz);
angular_velocity_d2 = y(id2_dottheta);
    
current_state=[x_init_d1; z_init_d1;theta_init_d1;xvelocity_init_d1; zvelocity_init_d1;angvelocity_init_d1;x_init_d2; z_init_d2;theta_init_d2;xvelocity_init_d2; zvelocity_init_d2;angvelocity_init_d2];
state =[current_state, [x_d1,z_d1,theta_d1,x_velocity_d1,z_velocity_d1,angular_velocity_d1,x_d2,z_d2,theta_d2,x_velocity_d2,z_velocity_d2,angular_velocity_d2]'];
control=[mode_diff_d1,mode_common_d1, mode_diff_d2,mode_common_d2];


% Run discrete prediction
ceq = [];
for i = 1:H
    ceq = [ceq; [(state(:,i+1) - timestep_euler(Ts,state(:,i), control(i,:)))]];
end


c=[];
end
