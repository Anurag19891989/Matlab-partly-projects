% Inverter Circuit Design and Simulation

% Clear previous variables and figures
clear;
clc;
close all;

% Define component values
R = 1000;         % Resistance in ohms
C = 1e-6;         % Capacitance in farads

% Define the input signal
Fs = 10000;       % Sampling frequency in Hz
T = 1/Fs;         % Sampling period
t = 0:T:0.01;     % Time vector (10 ms)
Vin = 5 * square(2*pi*50*t);  % Input square wave (50 Hz)

% Transfer function of RC circuit
s = tf('s');
H = 1 / (1 + R*C*s);

% Simulate the inverter response
Vout = lsim(H, Vin, t);

% Plot the input and output signals
figure;
subplot(2,1,1);
plot(t, Vin);
title('Input Signal (Square Wave)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

subplot(2,1,2);
plot(t, Vout);
title('Inverter Output Signal');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

% Inversion process: Ideally, for an inverter, Vout = Vcc - Vin.
% Simulate an ideal inverter
Vcc = 5;  % Supply voltage
Vout_ideal = Vcc - Vin;

% Plot the ideal inverter output
figure;
subplot(2,1,1);
plot(t, Vin);
title('Input Signal (Square Wave)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

subplot(2,1,2);
plot(t, Vout_ideal);
title('Ideal Inverter Output Signal');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;
