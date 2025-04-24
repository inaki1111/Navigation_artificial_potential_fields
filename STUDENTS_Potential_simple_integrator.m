%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

%----------- STUDENTS SIMULATION FILE ----------------

%% Navigation with artificial potential fields - simple integrator robot

% robot model
% p = [x;y] = 2D position
% simple integrator dynamics
% p_dot = [u1;u2]

% space state X_state = [x;y]
% X_dot = [u1;u2]

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
% robot state initial conditions
X_state=zeros(2,T);
% robot control input u
u=zeros(2,T);

% initial conditions (k=1)
X_state(:,1)=[2;1];

% scenario
% GOAL
% position of the goal
p_goal=[6;6];

% OBSTACLES
% position of obstacle 1
p_obs=[4;4];
% radius of obstacle 1 : if R_obs=0 the obstacle is a point with no dimensions
R_obs=0;
% position of obstacle 2
p_obs2=p_obs; % p_obs2=p_obs means that there is only one obstacle and not two at the same location
% radius of obstacle 2 : if R_obs2=0 the obstacle is a point with no dimensions
R_obs2=0;


% simulation
for k=2:T
    clf(); 
    hold on;
    axis([xmin xmax ymin ymax]); 
    axis square;

    % Gradient computation
    % gradient of the repulsive potential function
    dobs = distance_obs(X_state(:, k-1), p_obs, R_obs);
    grad_Urep= -dobs / norm(dobs)^3;
    
    % gradient of the attractive potential function
    grad_Uattr= 2*(X_state(:, k-1) - p_goal);

    % ponderation parameters
    alpha=1;
    beta=1;
    
    % total gradient = desired velocity vector
    grad_U=alpha*grad_Uattr + beta*grad_Urep;
    
    %control law
    u(:,k)=-grad_U;
        
    % robot state update using its dynamics
    f=robot_dynamics(X_state(:,k-1), u(:,k), 'integrator'); % use the function robot_dynamics to compute f
    X_state(:,k)=X_state(:,k-1) + f*dt; % use f to update X_state(:,k)

    % draw the potential field
    draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
    % draw the robot
    plot(X_state(1,k),X_state(2,k),'ored','LineWidth',3);
    % draw goal position
    plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
    % draw the obstacle
    draw_circular_obstacle(p_obs,R_obs);
    drawnow();
end

%% Figure
figure 
% draw field
draw_field(p_obs,p_obs2,p_goal,alpha,beta,limits);
hold on
axis([xmin xmax ymin ymax]); 
axis square;

% SCENARIO
% plot goal position
plot(p_goal(1),p_goal(2),'ogreen','LineWidth',4);
% plot obstacle position
plot(p_obs(1),p_obs(2),'oblack','LineWidth',3);

% ROBOT
% plot robot's initial state
% TO DO .........
% plot robot's trajectory
% TO DO .........
% plot robot's final state
% TO DO .........


