%----------- STUDENTS FILE ----------------
function x_dot=robot_dynamics(x,u,model)

switch model
    case 'integrator'
        % robot model: simple integrator
        % p = [x;y] = 2D position
        % simple integrator dynamics
        % p_dot = [u1;u2]
        % define space state X_space = [x;y]
        % x_dot = [u1;u2]

        x_dot=u;

    case 'unicycle'
        % robot model: unicycle
        % p = [x;y] = 2D position
        % theta = heading angle
        % v = speed vector
        % unicycle model
        % p_dot = [v*cos(theta);v*sin(theta)]
        % v_dot = u1
        % theta_dot = u2
        % define X_space = [x;y;v;theta]
        % x_dot = [v*cos(theta);v*sin(theta);u1;u2]   
        x_dot=[x(3)*cos(x(4));
               x(3)*sin(x(4));
               u(1);
               u(2)];
end
end

