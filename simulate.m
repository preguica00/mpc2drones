function simulate()

    close all
    clf
    hold on
    plot_prediction_d1 = plot(0,0,'or-', 'Linewidth', 1.5);
    plot_trajectory_d1 = plot(0,0,'db-','Linewidth', 1.5);
    
    plot_prediction_d2 = plot(0,0,'or-', 'Linewidth', 1.5);
    plot_trajectory_d2 = plot(0,0,'dk-','Linewidth', 1.5);
   
    % plot_uav_body = plot(0,0,'Color',[1 0.6 0],'LineWidth',3);
    state_trajectory=[];
    control_variables=[];
    
    axis square
    xlim([0 60])
    ylim([0 60])
    
    current_state = [0;0;0;0;0;0;0;0;0;0;0;0];
    current_MPC_solution = [];
    
 [H,Ts,id1_u1,id1_u2,id1_x,id1_z,id1_theta,id1_dotx,id1_dotz,id1_dottheta, id2_u1,id2_u2,id2_x,id2_z,id2_theta,id2_dotx,id2_dotz,id2_dottheta] = drones_info;
%     [xobs,yobs, obj_coord,radius] = obstacle;
    [mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
%     plot(xobs,yobs, '-k','Linewidth', 1.5);

    for k = 1:50
        
        %% Run the controller
        [command_d1, command_d2, current_MPC_solution, predicted_trajectory_d1,predicted_trajectory_d2] = ...
            optimizetrajectory(current_state, current_MPC_solution);
        
        %% Run the simulation
        current_state = simulate_timestep(current_state, command_d1,command_d2);

        %% Visualize
        plot_prediction_d1.XData = predicted_trajectory_d1(:,1);
        plot_prediction_d1.YData = predicted_trajectory_d1(:,2);
        plot_trajectory_d1.XData(end+1) = current_state(1);
        plot_trajectory_d1.YData(end+1) = current_state(2);
        
        plot_prediction_d2.XData = predicted_trajectory_d2(:,1);
        plot_prediction_d2.YData = predicted_trajectory_d2(:,2);
        plot_trajectory_d2.XData(end+1) = current_state(7);
        plot_trajectory_d2.YData(end+1) = current_state(8);
     %   plot_uav_body.XData = [0 -sin(current_state(3))]*4 + current_state(1);
     %   plot_uav_body.YData = [0 cos(current_state(3))]*4+ current_state(2);
        
        state_trajectory(end+1,:) = current_state;
        control_variables(end+1,:) = current_MPC_solution;
        
        drawnow
        pause(0.05)
    end
        figure
    plot(command_d1(:,1));
    hold on
    plot(command_d1(:,2));
     figure
    plot(command_d2(:,1));
    hold on
    plot(command_d2(:,2));
    
    save('states.mat','state_trajectory')
    save('control.mat','control_variables')

end
