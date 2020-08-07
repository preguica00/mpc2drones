function cost = costfunction(y, H)

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


%%com of drones
com_x = (x_d1+x_d2)/2;
com_z = (z_d1+z_d2)/2;

%vector variables of drones
vector_drone1=[10*mode_diff_d1; mode_common_d1; 50*theta_d1; x_velocity_d1 ;z_velocity_d1;angular_velocity_d1];
vector_drone2=[10*mode_diff_d2; mode_common_d2; theta_d2; x_velocity_d2 ;z_velocity_d2;angular_velocity_d2];

vector_drones=[com_x;com_z;vector_drone1;vector_drone2];

%distance between drones
d=5;
%reference
r=[60*ones(H,1);60*ones(H,1);zeros(H,1);10*ones(H,1);zeros(4*H,1);zeros(H,1);20*ones(H,1);zeros(4*H,1)];

d=5;

%cost = sum(vecnorm(vector_drones-r+ (x_d1 - x_d2).^2 + (z_d1 - z_d2).^2 - d.^2 ).^2);

cost = sum(vecnorm((x_d1 - x_d2).^2 + (z_d1 - z_d2).^2 - d.^2 ).^2);

end