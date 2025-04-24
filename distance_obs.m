%----------- STUDENTS FILE ----------------
function dpobs=distance_obs(p,p_obs,r)
    
    dpobs = (p - p_obs)*(norm(p - p_obs) - r) / norm(p - p_obs);

    %dpobs = p - p_obs - r*(p-p_obs) / norm(p - p_obs)^3

    % dpobs=[0;0];
end

