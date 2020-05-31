function cost = costfunction(y, H)

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


%     alpha=0.92; 
%     v=y;
%       for i=1:H
%           v=alpha*v+(1-alpha)*r;
%       end
%         cost =  sum((x(:)-v(k+i)).^2+(z(:)-v(k+i)).^2 +(theta(:)).^2 + (x_velocity(:)).^2+ (z_velocity(:)).^2+(angular_velocity(:)).^2 +50*(mode_diff(:)).^2+(mode_common(:)-common_final).^2);


%%com of drones
com_x = (x_d1+x_d2)/2;
com_z = (z_d1+z_d2)/2;

%vector variables of drones
vector_drone1=[10*mode_diff_d1; mode_common_d1; theta_d1; x_velocity_d1 ;z_velocity_d1;angular_velocity_d1];
vector_drone2=[10*mode_diff_d2; mode_common_d2; theta_d2; x_velocity_d2 ;z_velocity_d2;angular_velocity_d2];

vector_drones=[com_x;com_z;vector_drone1;vector_drone2];

%distance between drones
d=5;
%reference
r=[60*ones(H,1);60*ones(H,1);zeros(H,1);10*ones(H,1);zeros(4*H,1);zeros(H,1);10*ones(H,1);zeros(4*H,1)];

cost = sum(vecnorm(vector_drones-r).^2+(vecnorm(x_d1-x_d2).^2+ vecnorm(z_d1-z_d2).^2-d.^2).^2);



end