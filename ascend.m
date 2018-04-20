function ascend()
    disp("Connecting to robut.")
    accel_sub = rossubscriber('/accel');
    vel_pub = rospublisher('/raw_vel');
    
    cleanUp = @() stop(vel_pub);  % stop on exit/error
    cleanupObj = onCleanup(cleanUp);
    
    R = [0.944615793680593,0,-0.328178308742645;0,1,0;0.328178308742645,0,0.944615793680593];
    
    xt = 0.025;  % threshold for x values
    yt = 0.025;
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
        dist = vecnorm(g.*a);
        rot = atan2(g(2),g(1));
        % turn
        if abs(rot) > rt
            disp("Turning "+rad2deg(rot)+" degrees.")
            T = abs(rot / (tspeed*2/d));
            if rot > 0
                setVel(-tspeed,tspeed)
            else
                setVel(tspeed,-tspeed)
            end
            pause(T);
            setVel(0,0)
        end
        % drive
        disp("Driving "+dist+" meters.")
        T = (dist / mspeed);
        setVel(mspeed,mspeed)
        pause(T);
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