function ascend()
    disp("Connecting to robut.")
    accel_sub = rossubscriber('/accel');
    vel_pub = rospublisher('/raw_vel');
    
    cleanUp = @() stop(vel_pub);  % stop on exit/error
    cleanupObj = onCleanup(cleanUp);
    
    R = [0.952750292681088,0,-0.303754637489044;0,1,0;0.303754637489044,0,0.952750292681088];
    
    xt = 0.01;  % threshold for x values
    yt = 0.01;
    rt = pi/4;
    
    tspeed = 0.1;  % turn speed
    mspeed = 0.1;  % move speed
    
    d = 0.24;  % distance between wheels, in m
    a = 0.2;
    
    disp("Ready to climb.")
    disp("Press any key to start.")
    disp("Press CTRL+C to stop.")
    pause();  % wait for input
    disp("Starting climb.")

    flag = false;
    while ~flag
        % get gradient
        [x y z] = getAcceleration(R);
        if abs(x) < xt && abs(y) < yt
            disp("Leveled out.")
            disp("Waiting.")
            disp("x: "+x+"  y: "+y+"  z: "+z)
            pause(2)
            [x y z] = getAcceleration(R);
            if abs(x) < xt && abs(y) < yt
                disp("Stopping.")
                break
            else
                disp("Continuing.")
            end
        end
        g = -[x y];
        % determine angle to rotate and distance to drive
        
        T = g.*a;  % velocity vector
        T_hat = T ./ sqrt(sum(T.^2, 2));  % velocity unit vector
        N = diff(T_hat) ./ diff(u(1:end - 1));
        T_hat3 = [T_hat, zeros(size(T_hat(:, 1)))];  % add a third dim to T_hat
        N3 = [N, zeros(size(N(:, 1)))];  % add a third dim to N
        Omega = cross(T_hat3(1:end - 1, :), N3);  % rotational velocities
        V = sqrt(sum(T.^2, 2));  % linear velocities

        Vr = V(1:end-1,:) + d / 2 * sum(Omega, 2);
        Vl = V(1:end-1,:) - d / 2 * sum(Omega, 2);

        time = vecnorm(g)/V;
        setVel(Vl,Vr)
        pause(time);
        setVel(0,0);
%         pause(0.05)
    end
       
    function [x y z] = getAcceleration(R);
        accel = R * accel_sub.LatestMessage.Data;
        x = -accel(1);
        y = -accel(2);
        z = -accel(3);
    end

    function stop(pub)
        message = rosmessage(pub);
        message.Data = [0 0];
        send(pub, message);
    end

    function setVel(vl, vr)
        message = rosmessage(vel_pub);
        message.Data = [vl vr];
        send(vel_pub, message);
    end
end