% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz

% Calculate the wavelength
B = c / frequency;

% Define the doppler shifts in Hz using the information from above 
doppler_shifts = [3e3, -4.5e3, 11e3, -3e3];

% TODO: Calculate the velocity of the targets  fd = 2*vr/lambda
vr = doppler_shifts * B / 2;

% Display results
disp(vr);

range = 175 - vr * 5;
disp(range);