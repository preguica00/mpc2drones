function current_state = simulate_timestep(current_state,command)
 
    [H,Ts,drone_info] = drones_info;
    tspan = [0 Ts];

    my_ode = @(t,y) quadcopter_ode(t,y,command);
    [~, y] = ode45(my_ode,tspan, current_state);
    current_state = [y(end,1), y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7),y(end,8),y(end,9),y(end,10),y(end,11),y(end,12)];
end