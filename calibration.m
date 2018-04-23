% CALIBRATION  get current pitch and create a rotation matrix to correct it

accel_sub = rossubscriber('/accel');
pause(0.25)

accel = accel_sub.LatestMessage.Data;

x = accel(1);
y = accel(2);
z = accel(3);

% rotations
theta = tan(x/z);
R = getR(0,-theta,0);

corrected_a = R*accel;

function R = getR(roll,pitch,yaw)
    syms a b c
    Rx = [1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)];
    Ry = [cos(b), 0, sin(b); 0, 1, 0; -sin(b), 0, cos(b)];
    Rz = [cos(c), -sin(c), 0; sin(c), cos(c), 0; 0, 0, 1];
    Rsym = Rz*Ry*Rx;
    R = double(subs(Rsym,[a b c],[roll pitch yaw]));
end