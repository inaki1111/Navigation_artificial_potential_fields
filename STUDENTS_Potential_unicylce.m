%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

%----------- STUDENTS SIMULATION FILE ----------------

%% Navigation with artificial potential fields - non-holonomic robot

% robot model
% p = [x;y] = 2D position
% theta = heading angle
% v = speed vector
% unicycle car model
% p_dot = [v*cos(theta);v*sin(theta)]
% v_dot = u1
% theta_dot = u2
% define X_space = [x;y;v;theta]
% x_dot = [v*cos(theta);v*sin(theta);u1;u2]

% parameters for the figures
init;
xmin=0;
xmax=10;
ymin=0;
ymax=10;
limits=[xmin, xmax, ymin, ymax];

% simulation parameters
% sampling time
dt=0.01;
% number of iterations
T=300;

% robot variables
% robot state X_state
X_state=zeros(4,T);
% robot control input u
u=zeros(2,T);

% initial conditions (iteration k=1)
X_state(:,1)=[2;1;1;pi/4];

% scenario
% GOAL
% position of the goal
p_goal=[6;6];

% OBSTACLES
% position of obstacle 1
p_obs=[4;4];
% radius of obstacle 1 : if R_obs=0 the obstacle is a point with no dimensions
R_obs=0.2;
% position of obstacle 2
p_obs2=[6;4]; % p_obs2=p_obs means that there is only one obstacle and not two at the same location
% radius of obstacle 2 : if R_obs2=0 the obstacle is a point with no dimensions
R_obs2=0.2;

% simulation
for k=2:T
    clf(); 
    hold on;
    axis([xmin xmax ymin ymax]); 
    axis square;

    % Gradient computation
    % gradient of the repulsive potential function 1 obstacle
    %grad_Urep=0;
    dobs = distance_obs(X_state(1:2, k-1), p_obs, R_obs);
    grad_Urep= -dobs / norm(dobs).^3;

    dobs2 = distance_obs(X_state(1:2, k-1), p_obs2, R_obs);
    grad_Urep2= -dobs2 / norm(dobs2).^3;

    % gradient of the attractive potential function
    %grad_Uattr=0;
    grad_Uattr= 2*(X_state(1:2, k-1) - p_goal);

    % ponderation parameters
    alpha=1;
    beta=1;
    
    % total gradient = desired velocity vector
    %grad_U=;
    grad_U = beta*grad_Urep + alpha*grad_Uattr + beta*grad_Urep2;
    
    % reference velocity vector
    ref_vel_vec=-grad_U;
    % desired speed vector
    vref=norm(ref_vel_vec);
    % desired heading angle
    thetaref=atan2(ref_vel_vec(2), ref_vel_vec(1));

    %control law
    %control parameters
    K1=1;
    K2=10;
    %control input
    u(:,k-1)=[-K1*(X_state(3, k-1)-vref); -K2*atan(tan(X_state(4, k-1)/2 - thetaref/2))];
          
    % robot state update using its dynamics
    f=robot_dynamics(X_state(:,k-1), u(:,k-1), 'unicycle'); % use the fuction robot_dynamics to compute f
    X_state(:,k)=X_state(:,k-1)+ f*dt; % use f to update X_space(:,k)

    % draw the potential field
    draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
    % draw the robot
    draw_robot(X_state([1,2,4],k),'red',0.1);
    % draw goal position
    plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
    % draw the obstacle(s)
    draw_circular_obstacle(p_obs,R_obs);
    draw_circular_obstacle(p_obs2,R_obs2);
    drawnow();
end

%% Figures
figure 
draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
hold on
axis([xmin xmax ymin ymax]); 
axis square;

% SCENARIO
% plot goal position
plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
% plot the obstacle(s)
draw_circular_obstacle(p_obs,R_obs);
draw_circular_obstacle(p_obs2,R_obs2);

% ROBOT
% plot robot's initial state
% TO DO .........
% plot robot's trajectory
plot(X_state(1,:), X_state(2,:))
% plot robot's final state
draw_robot(X_state([1,2,4],end))
