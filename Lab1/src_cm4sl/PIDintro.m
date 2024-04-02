%% PID intro 
close all

%% mass, spring and damper values and transfer function
M = 20;                     % mass in kg
K = 400;                    % spring stiffness coeficient in N/m
C = 200;                    % damping coefficient in Ns/m    
s = tf('s');                % LaPlace parameter
t = 0:0.01:2;               % time vector

%% The PID values you will adjust
KP = 1;
KI = 0;
KD = 0;

%% implement your transfer function (T) 
T = 1/(s^2*M+s*C+K);

%% Implement controller and run a step response with F = 400 N
Ctrl = pid(KP,KI,KD);
S = feedback(Ctrl*T,400);
[response, times] = step(S,t);

%% Characteristics
info = stepinfo(S);

%% Compute the derivatives of the response
dResponse = diff(response)./diff(times);
ddResponse = diff(dResponse)./diff(times(1:end-1));

%% Get inflection point
absddResponse = abs(ddResponse);

PeakTimeIndex = info.PeakTime/0.01;
roundedPeakTimeIndex = round(PeakTimeIndex);

[inflectionPoint, inflectionIndex] = min(absddResponse(1:roundedPeakTimeIndex));
inflectionTime = times(inflectionIndex);

%% Get tangent line at inflection point
gradient = dResponse(inflectionIndex);
offset = response(inflectionIndex) - gradient * t(inflectionIndex);

tangent = @(x) gradient*x + offset;

%% Tangentline from time-axis to steady-state-line
zerocrossing = -offset/gradient;
steadystatevalue = response(end);
steadystatereached = (steadystatevalue-offset) / gradient; 

tangenttime = zerocrossing:0.01:steadystatereached;

%% L and T values
Lvalue = zerocrossing;
Tvalue = steadystatereached - zerocrossing;

%% Create figure
figure;

% lines
Response = plot(times, response, 'b', 'DisplayName', 'Response', 'LineWidth', 3);
hold on;
TangentAtInflectionPoint = plot(tangenttime, tangent(tangenttime), 'DisplayName', 'Tangent at Inflection Point', 'LineStyle', '--', 'Color', 'k', 'LineWidth', 1.5);
height = steadystatevalue / 2;
Lline = line([0, zerocrossing], [height, height], 'LineStyle', '-', 'Color', 'r', 'DisplayName', 'L Value', 'LineWidth', 3);
Tline = line([zerocrossing, steadystatereached], [height, height], 'LineStyle', '-', 'Color', 'g', 'DisplayName', 'T Value', 'LineWidth', 3);
Lhorizontal = line([zerocrossing, zerocrossing], [0, steadystatevalue], 'LineStyle', '--', 'Color', 'k');
Thorizontal = line([steadystatereached, steadystatereached], [0, steadystatevalue], 'LineStyle', '--', 'Color', 'k');

% y lines
SteadyStateLine = yline(steadystatevalue, 'LineStyle', '--', 'Color', 'b', 'DisplayName', 'Steady State Line', 'LineWidth', 1.5);

% points
plot(zerocrossing, 0, 'ro', 'LineWidth', 1.5)
plot(steadystatereached, steadystatevalue, 'ro', 'LineWidth', 1.5)
InflectionPoint = plot(inflectionTime, response(inflectionIndex), 'rx', 'DisplayName', 'Inflection Point', 'LineWidth', 3);

% Legend
legend([Response, SteadyStateLine, TangentAtInflectionPoint, Lline, Tline, InflectionPoint], 'Location','southeast')

% Axis lables
xlabel('Time [s]')
ylabel('Amplitude')

hold off

%% Ziegler-Nichols Parameters
% P-Controller
KP_P = 0.9 * (Tvalue/Lvalue);
KI_P = 0;
KD_P = 0;

% PI-Controller
KP_PI = 0.9 * (Tvalue/Lvalue);
KI_PI = 0.27 * (Tvalue/Lvalue^2);
KD_PI = 0;

% PID_Controller
KP_PID = 1.2 * (Tvalue/Lvalue);
KI_PID = 0.6 * (Tvalue/Lvalue^2);
KD_PID = 0.6 * Tvalue;

%% System Response and Plot for each Controller
Ctrl_P = pid(KP_P,KI_P,KD_P);
Ctrl_PI = pid(KP_PI,KI_PI,KD_PI);
Ctrl_PID = pid(KP_PID,KI_PID,KD_PID);

S_P = feedback(Ctrl_P*T,400);
S_PI = feedback(Ctrl_PI*T,400);
S_PID = feedback(Ctrl_PID*T,400);

figure;

ref = step(S,t);
hold on 
P = step(S_P, t);
PI = step(S_PI, t);
PID = step(S_PID, t);

plot(times, ref, 'LineWidth', 3)
plot(times, P, 'LineWidth', 3)
plot(times, PI, 'LineWidth', 3)
plot(times, PID, 'LineWidth', 3)

xlabel('Time [s]')
xlabel('Amplitude')


legend('KP = 1', 'P', 'PI', 'PID')
hold off

%% Improved Controllers
% P-Controller-Improved
KP_P_Improved = (0.9 * (Tvalue/Lvalue)) * 1.7 %slight overshoot but reaches steady state faster
KI_P_Improved = 0;
KD_P_Improved = 0;
% PI-Controller-Improved
KP_PI_Improved = (0.9 * (Tvalue/Lvalue)) * 3.5
KI_PI_Improved = (0.27 * (Tvalue/Lvalue^2)) * 1.25
KD_PI_Improved = 0;
% PID_Controller-Improved
KP_PID_Improved = (1.2 * (Tvalue/Lvalue)) * 5.5;
KI_PID_Improved = (0.6 * (Tvalue/Lvalue^2)) * 1.5 ;
KD_PID_Improved = (0.6 * Tvalue) * 8.5;

% Controller
Ctrl_P_Improved = pid(KP_P_Improved,KI_P_Improved,KD_P_Improved);
Ctrl_PI_Improved = pid(KP_PI_Improved,KI_PI_Improved,KD_PI_Improved);
Ctrl_PID_Improved = pid(KP_PID_Improved,KI_PID_Improved,KD_PID_Improved);
% Step Response with F=400N
S_P_Improved = feedback(Ctrl_P_Improved*T,400);
S_PI_Improved = feedback(Ctrl_PI_Improved*T,400);
S_PID_Improved = feedback(Ctrl_PID_Improved*T,400);
P_Improved = step(S_P_Improved,t);
PI_Improved = step(S_PI_Improved,t);
PID_Improved = step(S_PID_Improved,t);


figure
plot(times, P, 'DisplayName', 'P')
hold on
plot(times, P_Improved, 'DisplayName', 'P improved')
legend('show')
hold off

figure
plot(times, PI, 'DisplayName', 'PI')
hold on
plot(times, PI_Improved, 'DisplayName', 'PI improved')
legend('show')
hold off

figure
plot(times, PID, 'DisplayName', 'PID')
hold on
plot(times, PID_Improved, 'DisplayName', 'PID imporved')
legend('show')
hold off
%plot(times, P_Improved, 'DisplayName', 'P improved')

legend('show')
