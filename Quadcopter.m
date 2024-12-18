%Defining Parameters
m = 0.5;  %Mass of the Quadcopter(Assuming 1 kilogram)
g = [0; 0; -9.81];  %Acceleration due to gravity
kf = 6.41e-6; %Thrust coefficient
kt = 1.69e-2; %Torque coefficient
gamma = 2.75e-3; %Drag coefficient
l = 0.17; %Distance from COM to centre of propeller
d1 = 0; d2 = 0; d3 = 0;
d = [d1; d2; d3];   %Position of COM w.r.t. inertial frame
v = 0; %Initial Velocity
psi = 0; %Yaw
theta = 0; %Pitch
phi = 0; %roll
omega = [0; 0; 0]; %Angular velocities([p, q, r])
%Rotation matrix(vehicle frame to body frame)
R = [
     cos(psi)*cos(theta)-(sin(phi)*sin(psi)*sin(theta)) -cos(phi)*sin(psi) cos(psi)*sin(theta) + (cos(theta)*sin(phi)*sin(psi))
     cos(theta)*sin(psi) + (cos(psi)*sin(phi)*sin(theta)) cos(phi)*cos(psi) sin(psi)*sin(theta) - (cos(psi)*cos(theta)*sin(phi))
     -cos(phi)*sin(theta)                                  sin(phi)          cos(phi)*cos(theta)];
display(R);
%%Don't forget to input the inertia  and torque values%%
ITxx = 3.2e-3; ITyy = ITxx; ITzz = 5.5e-3;
IT = [ITxx 0 0; 0 ITyy 0; 0 0 ITzz]; %Total body moment of inertia
IPxx = 0; IPyy = 0; IPzz = 1.5e-5; 
IP = [IPxx 0 0; 0 IPyy 0; 0 0 IPzz]; %Propeller's rotational inertia(in body frame)
Td = [0; 0; -gamma*omega(3)]; %Drag Torque(simplified: drag only affects yaw rate(r))

omega_rotors = [400; 400; 400; 400]; %Initial rotor speeds

%Forces and torques
lifts = kf * omega_rotors.^2; %Lifts of each propeller
total_lift = sum(lifts);
torque_resultant = [(lifts(2)-lifts(4)*l);
                    (lifts(3)-lifts(1)*l);
                    (lifts(1)-lifts(2)+lifts(3)-lifts(4)*kt)] + Td;
%--------Dynamics--------%

%Time settings
dt = 0.01; %0.01sec
T = 10; %10sec simulation time
N = T/dt; %No of samples
t = 0:dt:T; %Time Vector
for i=1:N
    %Translational
    %v(t+dt) = v(t) + a*dt; d(t+dt) = d + v*dt;
    thrust_in_inertial = R * [0; 0; total_lift];
    a = g + (thrust_in_inertial/m);
    v = v + a * dt;
    d = d + v * dt;
    %Rotational
    %w(t+dt) = w(t) + alpha*dt; R' = -wB x R(=> R(t+dt) = e^-(-wB * dt)*R(t));
    alpha = [
    (1 / ITxx) * (torque_resultant(1) - (ITyy - ITzz) * omega(2) * omega(3));
    (1 / ITyy) * (torque_resultant(2) - (ITzz - ITxx) * omega(3) * omega(1));
    (1 / ITzz) * (torque_resultant(3) - (ITxx - ITyy) * omega(1) * omega(2))
];
    omega = omega + alpha * dt;
    omega_skew = [0,         -omega(3),  omega(2);
                  omega(3),   0,        -omega(1);
                 -omega(2),   omega(1),  0 ];
    R = R * expm(omega_skew * dt);  % Update orientation matrix
    % Store history
    angular_velocity_history(:, i) = omega;
    position_history(:, i) = d;
    orientation_history(:, :, i) = R;

end
figure
plot3(position_history(1, :), position_history(2, :), position_history(3, :));
grid on;
title('Quadcopter Position');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');