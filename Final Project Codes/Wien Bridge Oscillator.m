% Wien Bridge Oscillator Design and Simulation

% Clear previous variables and figures
clear;
clc;
close all;

% Define component values
R1 = 10000;     % Resistance in ohms
R2 = 10000;     % Resistance in ohms
C1 = 0.01e-6;   % Capacitance in farads
C2 = 0.01e-6;   % Capacitance in farads
R3 = 10000;     % Resistance in ohms (for feedback)
R4 = 10000;     % Resistance in ohms (for feedback)

% Calculate the oscillation frequency
f_osc = 1 / (2 * pi * sqrt(R1 * R2 * C1 * C2));
fprintf('Oscillation Frequency: %.2f Hz\n', f_osc);

% Transfer Function of Wien Bridge Oscillator
s = tf('s');
H = (R4/R3) * (s^2 + 1/(R1*C1)) / (s^2 + (1/(R1*C1) + 1/(R2*C2))*s + 1/(R1*R2*C1*C2));

% Display the transfer function
disp('Transfer Function H(s):');
disp(H);

% Frequency response (Bode plot)
figure;
bode(H);
grid on;
title('Bode Plot of Wien Bridge Oscillator');

% Time domain simulation
% Generate a time vector and input signal
t = 0:1e-6:0.005;  % Time vector (5 ms)
u = sin(2*pi*f_osc*t);  % Input signal (sine wave at oscillation frequency)

% Simulate the oscillator response
[y, t] = lsim(H, u, t);

% Plot the input and output signals
figure;
subplot(2,1,1);
plot(t, u);
title('Input Signal (Sine Wave)');
xlabel('Time (s)');
ylabel('Amplitude');

subplot(2,1,2);
plot(t, y);
title('Oscillator Output Signal');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

% Note: In a real Wien Bridge Oscillator circuit, non-linear elements like
% diodes or lamp are used to stabilize the amplitude. This simple model
% does not include those elements.
