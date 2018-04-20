function ascend()
    % drive up until stopped or level
    disp("Connecting to robut.")
    accel_sub = rossubscriber('/accel');
    spub = rospublisher('/raw_vel');
    
    cleanUp = @() setVel(0,0,spub);
    cleanupObj = onCleanup(cleanUp);
    
    xt = 0.025;  % threshold for x values
    yt = 0.05;
    
    tspeed = 0.1;  % turn speed
    mspeed = 0.1;  % move speed
    
    R = [0.944615793680593,0,-0.328178308742645;0,1,0;0.328178308742645,0,0.944615793680593];
    
    disp("Ready to climb.")
    disp("Press any key to start.")
    disp("Press CTRL+C to stop.")
    pause();  % wait for input
    disp("Starting climb.")
    flag = false;
    while ~flag
        [x,y,z] = getAcceleration(R,accel_sub);
        
        if y > yt
            % rotate cw
            setVel(tspeed,-tspeed,spub)
        elseif y < -yt
            % rotate ccw
            setVel(-tspeed,tspeed,spub)
        elseif x > xt
            % back it up
            setVel(-mspeed,-mspeed,spub)
        elseif x < -xt
            % move forward
            setVel(mspeed,mspeed,spub)
        else
            disp("Leveled out.")
            disp("x: "+x+"  y: "+y+"  z: "+z)
            setVel(0,0,spub)
            flag = true;  % stop
        end
        pause(0.2);
    end
    
    function [x y z] = getAcceleration(R,sub);
        accel = R * sub.LatestMessage.Data;
        x = -accel(1);
        y = -accel(2);
        z = -accel(3);
    end

    function setVel(vl, vr, pub)
        message = rosmessage(pub);
        message.Data = [vl vr];
        send(pub, message);
    end
end