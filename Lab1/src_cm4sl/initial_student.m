%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KTH SD2231 - Applied Vehicle Control
%
%  Lab:     1 - Slip Control for vehicles
%  Date:    Spring term 2020
%  Teacher: Mikael Nybacka
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
close all

global Veh
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Choose input parameters
mu_select = 1;              % set friction to mu_select = 1 (dry road), 2 (wet 
                            % road) or 3 (snow)
dt = 0.001;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle parameters DO NOT CHANGE
% ______________________________________________________________________________
% 
% BASIC PARAMETERS         UNIT          TOTAL
% ______________________________________________________________________________
% 
% Total mass               kg           2107.996
% Total weight             N           20671.009
% Total roll inertia       kgm^2         952.228
% Total pitch inertia      kgm^2        3508.505
% Total yaw inertia        kgm^2        3954.288
% Total CoG height (FrD)   m               0.545
% 
% Sprung mass              kg           1841.860
% Sprung weight            N           18061.279
% Sprung roll inertia      kgm^2         741.760
% Sprung pitch inertia     kgm^2        2903.000
% Sprung yaw inertia       kgm^2        3170.600
% Sprung CoG height (FrD)  m               0.578
% 
% Unsprung mass (total)    kg            266.136
% 
% Wheelbase                m               2.971
% 
% ______________________________________________________________________________
% 
% BASIC PARAMETERS         UNIT      AVERAGE/TOTAL       LEFT         RIGHT
% ______________________________________________________________________________
% 
% Front axle:
% 
% Track width              m               1.662
% Toe Angle                deg             0.200          0.200         0.200
% Camber Angle             deg            -0.000         -0.000        -0.000
% 
% Caster Angle             deg             1.298          1.298         1.298
% Caster Offset            mm              6.077          6.077         6.077
% Kingpin Angle            deg            13.257         13.257        13.257
% Kingpin Offset           mm              2.809          2.809         2.809
% 
% Normal force             N           10436.920       5218.459      5218.461
% Loaded tire radius       m               0.296          0.296         0.296
% Tire rate                N/mm          236.700        236.700       236.700
% ______________________________________________________________________________
% 
% Rear axle:
% 
% Track width              m               1.715
% Toe Angle                deg             0.115          0.115         0.115
% Camber Angle             deg            -1.490         -1.490        -1.490
% 
% Normal force             N           10234.057       5117.028      5117.029
% Loaded tire radius       m               0.297          0.297         0.297
% Tire rate                N/mm          236.700        236.700       236.700
% ______________________________________________________________________________

% Initial values for road vehicle
         g = 9.81;
         Veh.ms  = 1841.860;            % sprung mass of the car in kg 
         Veh.mw = 266.136;              % unsprung mass of the car in kg
         Veh.Jw = 2.592;                % inertia of wheel in kgm^2
         Veh.L = 2.971;                 % Wheelbase
         Veh.lambda = 0.4958;           % Ratio of CoG
         Veh.Lf = Veh.L*Veh.lambda;     % Distance from front axis to CoG
         Veh.Lr = Veh.L*(1-Veh.lambda); % Distance from front axis to CoG
         Veh.h = 0.545;                 % CoG height from ground
         Veh.r = 0.297;                 % rolling radius for tire in m
         
        Veh.mu = [0.9 0.6 0.3];           % friction coefficients [dry, rain and snow]
        K_em = [350 160 80];
        K_brake = [650 500 400];
        tire_leg={'\mu = 1','\mu = 0.7','\mu = 0.3'};
        switch mu_select
            case {1}
                K_em = K_em(1)
                K_brake = K_brake(1)
                Veh.mu = Veh.mu(1)
            case {2}
                K_em = K_em(2)
                K_brake = K_brake(2)
                Veh.mu = Veh.mu(2)
            case {3}
                K_em = K_em(3)
                K_brake = K_brake(3)
                Veh.mu = Veh.mu(3)
        end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here you can ad your control parameters for the electric motor

Kp_em = 0;
Ki_em = 0;
Kd_em = 0;


% Here you can ad your control parameters for the brake 

Kp_brake = 0;
Ki_brake = 0;
Kd_brake = 0;

% Run your model
%sim('SD2231_Lab1_2024.slx')


%% Calculating the mu's
time = 1151:1:1500;
mu_fl = FxFL.Data(1:8000)./FzFL.Data(1:8000);
mu_rl = FxRL.Data(time)./FzRL.Data(time);

[mu_rl_max, index] = max(mu_rl);
slip_max = slips.signal1.Data(1150+index)


%% Ploting mu over traction slip
%figure
%plot(slips.signal1.Data(1000:1000+length(time)),  FxRL.Data(1000:1000+length(time))./FzRL.Data(1000:1000+length(time)))

%figure
%plot(mu_rl, slips.signal1.Data(time))
plot(slips.signal1.Data(time), mu_rl, 'LineWidth', 3)
hold on
plot(slip_max, mu_rl_max, 'ro', 'LineWidth', 3)

xlabel("Traction Slip s")
ylabel("Friction coefficient mu")
title("Friction over Traction Slip, RL Tire")
hold off

%figure
%plot(time, slips.signal1.Data(1001:1000+length(time)))

%figure
%plot(time, FxRL.Data(1001:1000+length(time))./FzRL.Data(1001:1000+length(time)))

%% Plotin mu over braking slip RL

time = 8000:1:8250;
%mu_fl = FxFL.Data(1:8000)./FzFL.Data(1:8000);
mu_rl = abs(FxRL.Data(time)./FzRL.Data(time));

[mu_rl_max, index] = max(mu_rl);
slip_max = slips.signal2.Data(7999+index)

figure
plot(slips.signal2.Data(time), mu_rl)
hold on
plot(slip_max, mu_rl_max, 'ro', 'LineWidth', 3)

xlabel("Braking Slip s")
ylabel("Friction coefficient mu")
title("Friction over Braking Slip, RL Tire")
hold off

%% Ploting mu over breaking slip FL

time = 8000:1:8250;
%mu_fl = FxFL.Data(1:8000)./FzFL.Data(1:8000);
mu_fl = abs(FxFL.Data(time)./FzFL.Data(time));

[mu_fl_max, index] = max(mu_fl);
slip_max = slips.signal2.Data(7999+index)

figure
plot(slips.signal2.Data(time), mu_fl)
hold on
plot(slip_max, mu_fl_max, 'ro', 'LineWidth', 3)

xlabel("Braking Slip s")
ylabel("Friction coefficient mu")
title("Friction over Braking Slip, FL Tire")
hold off
