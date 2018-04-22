accel_sub = rossubscriber('/accel');
accel = zeros(3,1);
pause(0.25)



while true
    accel = R*accel_sub.LatestMessage.Data;
    x = accel(1);
    y = accel(2);
    z = accel(3);
    disp("x: "+x+"  y: "+y+"  z: "+z)
    pause(0.25);
end
