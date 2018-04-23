function ascend()
% ASCEND  perform gradient ascent IRL
    disp("Connecting to robut.")
    accel_sub = rossubscriber('/accel');
    vel_pub = rospublisher('/raw_vel');
    
    cleanUp = @() stop(vel_pub);  % stop on exit/error
    cleanupObj = onCleanup(cleanUp);
    
    R = [0.934989503350951,0,-0.354675384857114;0,1,0;0.354675384857114,0,0.934989503350951];
    
    xt = 0.01;  % threshold for x values
    yt = 0.01;  % threshold for y values
    rt = pi/4;  % threshold for rotation values
    
    tspeed = 0.2;  % turn speed
    mspeed = 0.15;  % move speed
    
    d = 0.25;  % distance between wheels, in m
    a = 0.2;   % constant for gradient acceleration
    
    disp("Ready to climb.")
    disp("Press any key to start.")
    disp("Press CTRL+C to stop.")
    pause();  % wait for input
    disp("Starting climb.")

    flag = false;
    while ~flag
        [x,y,z] = getAcceleration(R);
        disp("x: "+x+"  y: "+y+"  z: "+z)
        if abs(x) < xt && abs(y) < yt  % check if "level"
            disp("Currently level.")
            setVel(0,0)
            pause;  % wait for input
            setVel(0.1,0.1)  % move forward
            pause(2)
% %             setVel(-0.2,0.2)
% %             pause(1)
% %             setVel(0.1,0.1)
% %             pause(1)
% %             setVel(0,0)
% %             pause(4)
%             [x,y,z] = getAcceleration(R);
%             if abs(x) < xt && abs(y) < yt
%                 disp("Leveled out.")
%                 disp("x: "+x+"  y: "+y+"  z: "+z)
%                 break
%             end
            [x,y,z] = getAcceleration(R);  % update readings
        end
        w = remap(-y,[0,0.3],[0.05,0.3]);  % force minimum rotation
%         v = remap(-x,[0,0.3],[0,mspeed]);
%         w = -y;
        v = -x*0.5;  % reduce linear velocity
%         v = 0;
        if abs(w) > 0.3
            w = 0.3 * w/abs(w);
        end
        if abs(v) > 0.3
            v = 0.3 * v/abs(v);
        end
        % separate turning and driving
%         v = -x/abs(-x)*0.1;
%         w = -y*4;
        % turn
%         Vr = d/2*w;
%         Vl = -d/2*w;
%         setVel(Vl,Vr);
%         pause(w*10)
        % drive
%        Vr = v;
%        Vl = v;
%        setVel(Vl,Vr)
%        pause(v*10)
        Vr = v + d / 2 * w;
        Vl = v - d / 2 * w;
        
        setVel(Vl,Vr)
        pause(0.1);
    end
       
    function [x y z] = getAcceleration(R);
    % GETACCELERATION  get current accelerometer data
        C = [-0.025; -0.003; -1.038];  % constant for normalizing
        accel = accel_sub.LatestMessage.Data;
        % invert accelerometer readings
        x = -accel(1);
        y = -accel(2);
        z = -accel(3);
    end

    function stop(pub)
    % STOP  stop the robot
        message = rosmessage(pub);
        message.Data = [0 0];
        send(pub, message);
    end

    function setVel(vl, vr)
    % SETVEL  set the wheel velocities
        message = rosmessage(vel_pub);
        message.Data = [vl vr];
        send(vel_pub, message);
    end
end

function z = remap(c,ab,xy)
% REMAP  map values from one range to another
    a = ab(1);
    b = ab(2);
    x = xy(1);
    y = xy(2);
    z = c/abs(c)*((abs(c) - a) / (b-a) * (y - x) + x);
end