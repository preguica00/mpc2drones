function current_state = simulate_timestep(current_state,command_d1,command_d2)
 [H,Ts,id1_u1,id1_u2,id1_x,id1_z,id1_theta,id1_dotx,id1_dotz,id1_dottheta, id2_u1,id2_u2,id2_x,id2_z,id2_theta,id2_dotx,id2_dotz,id2_dottheta] = drones_info;
    tspan = [0 Ts];

    command=[command_d1;command_d2];
    my_ode = @(t,y) quadcopter_ode(t,y,command);
    [~, y] = ode45(my_ode,tspan, current_state);
    current_state = [y(end,1), y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7),y(end,8),y(end,9),y(end,10),y(end,11),y(end,12)];
end