load ('states.mat')
load ('control.mat')

figure
subplot(3,2,1)
plot(state_trajectory(:,1), 'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('x (m)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,2)
plot(state_trajectory(:,2),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('z (m)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,3)
plot(rad2deg(state_trajectory(:,3)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('\theta (º)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,4)
plot(state_trajectory(:,4),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{x}$ (m/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,5)
plot(state_trajectory(:,5),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{z}$ (m/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,6)
plot(rad2deg(state_trajectory(:,6)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{\theta}$ (rad/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
subplot(1,2,1)
plot(control_variables(:,1),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$u_{1}$ (N)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)
subplot(1,2,2)
plot(control_variables(:,2),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$u_{2}$ (N)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

% subplot(2,2,3)
% plot(state_trajectory(:,7),'b', 'Linewidth', 1.5);
% xlim([1 50])
% ylabel('$f_1$ (N)', 'Interpreter','latex')
% xlabel('Discrete Time')
% 
% subplot(2,2,4)
% plot(state_trajectory(:,8),'b', 'Linewidth', 1.5);
% xlim([1 50])
% ylabel('$f_2$ (N)', 'Interpreter','latex')
% xlabel('Discrete Time')


%% Drone 2

figure
subplot(3,2,1)
plot(state_trajectory(:,7), 'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('x (m)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,2)
plot(state_trajectory(:,8),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('z (m)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,3)
plot(rad2deg(state_trajectory(:,9)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('\theta (º)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,4)
plot(state_trajectory(:,10),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{x}$ (m/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,5)
plot(state_trajectory(:,11),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{z}$ (m/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

subplot(3,2,6)
plot(rad2deg(state_trajectory(:,12)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{\theta}$ (rad/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
subplot(1,2,1)
plot(control_variables(:,3),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$u_{1}$ (N)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)
subplot(1,2,2)
plot(control_variables(:,4),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$u_{2}$ (N)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)


% subplot(2,2,3)
% plot(state_trajectory(:,7),'b', 'Linewidth', 1.5);
% xlim([1 50])
% ylabel('$f_1$ (N)', 'Interpreter','latex')
% xlabel('Discrete Time')
% 
% subplot(2,2,4)
% plot(state_trajectory(:,8),'b', 'Linewidth', 1.5);
% xlim([1 50])
% ylabel('$f_2$ (N)', 'Interpreter','latex')
% xlabel('Discrete Time')
