% Ray Zhang
classdef crazyflie < handle
 
    
    properties (SetAccess = immutable)
        mass = 0.030;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
        g = 9.81;   % gravitational constant
        I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
            0,         1.43e-5,    0;
            0,         0,          2.89e-5];
        arm_Length = 0.046; % arm length in m
        invI = inv([1.43e-5,   0,          0; % inertial tensor in m^2 kg
            0,         1.43e-5,    0;
            0,         0,          2.89e-5]);
        rotor_speed_min = 0;    % rad/s
        rotor_speed_max= 2500;  % rad/s
        k_thrust = 2.3e-08;     % N/(rad/s)**2
        k_drag = 7.8e-11;       % Nm/(rad/s)**2
        
    end
    
    methods
        
    end
end

