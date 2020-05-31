  function  [command_d1, command_d2, optimum, predicted_trajectory_d1,predicted_trajectory_d2] = ...
    optimizetrajectory(current_state, optimum)


    % Initial conditions drone 1
    x_init_d1 = current_state(1);
    z_init_d1 = current_state(2);
    theta_init_d1 = current_state(3);
    xvelocity_init_d1 = current_state(4);
    zvelocity_init_d1 = current_state(5);
    angvelocity_init_d1 = current_state(6);
    
    % Initial conditions drone 2

    x_init_d2 = current_state(7);
    z_init_d2 = current_state(8);
    theta_init_d2 = current_state(9);
    xvelocity_init_d2 = current_state(10);
    zvelocity_init_d2 = current_state(11);
    angvelocity_init_d2 = current_state(12);
    

    [H,Ts,id1_u1,id1_u2,id1_x,id1_z,id1_theta,id1_dotx,id1_dotz,id1_dottheta, id2_u1,id2_u2,id2_x,id2_z,id2_theta,id2_dotx,id2_dotz,id2_dottheta] = drones_info

     
    %initial conditions
    if isempty(optimum)
        optimum = [zeros(H,1);10*ones(H,1);zeros(6*H,1);zeros(H,1);10*ones(H,1);zeros(6*H,1)];
    end

    opts = optimoptions('fmincon','Algorithm','sqp','TolFun',0.001,'MaxIter',100000,'MaxFunEvals',100000);



   [optimum, ~] = fmincon(@(y)costfunction(y, H), optimum,[],[],[],[],[],[],@(y)discretization(y,x_init_d1,z_init_d1,theta_init_d1,xvelocity_init_d1,zvelocity_init_d1,angvelocity_init_d1,x_init_d2,z_init_d2,theta_init_d2,xvelocity_init_d2,zvelocity_init_d2,angvelocity_init_d2),opts);
   
   %optimal control drone 1
   u1_optimum_d1 = optimum(id1_u1);
   u2_optimum_d1 = optimum(id1_u2);
   command_d1 = [u1_optimum_d1(1), u2_optimum_d1(1)];

    x_optimum_d1 = optimum(id1_x);
    z_optimum_d1 = optimum(id1_z);
    theta_optimum_d1 = optimum(id1_theta);
    xvelocity_optimum_d1 = optimum(id1_dotx);
    zvelocity_optimum_d1 = optimum(id1_dotz);
    angvelocity_optimum_d1 = optimum(id1_dottheta);
    
    %optimal control drone 2
    u1_optimum_d2 = optimum(id2_u1);
    u2_optimum_d2 = optimum(id2_u2);
    command_d2 = [u1_optimum_d2(1), u2_optimum_d2(1)];
   
    %optimal states drone 2
    x_optimum_d2 = optimum(id2_x);
    z_optimum_d2 = optimum(id2_z);
    theta_optimum_d2 = optimum(id2_theta);
    xvelocity_optimum_d2 = optimum(id2_dotx);
    zvelocity_optimum_d2 = optimum(id2_dotz);
    angvelocity_optimum_d2 = optimum(id2_dottheta);
    
    %predicted trajectory of drones
    predicted_trajectory_d1 = [x_optimum_d1,z_optimum_d1,theta_optimum_d1,xvelocity_optimum_d1,zvelocity_optimum_d1,angvelocity_optimum_d1];
    predicted_trajectory_d2 = [x_optimum_d2,z_optimum_d2,theta_optimum_d2,xvelocity_optimum_d2,zvelocity_optimum_d2,angvelocity_optimum_d2];

  
  end

