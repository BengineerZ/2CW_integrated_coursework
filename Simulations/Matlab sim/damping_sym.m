close all
clear all

% floor properties:
M = 1.83;
K = 4200;
L = 0.54;

% Number of floors:
N = 3;

% TMD properties:
m = 0.15;
%k = 56;
k = 59.7;
l = 0.6;


% Generate the mass matrix for N floors
mass_array = zeros(1,N+1);
for i=1:N
  mass_array(i) = M;
endfor
mass_array(N+1) = m;
mass_array
mass_array = [M,M,M+0.5,m];

mass_matrix = diag(mass_array);
mass_matrix

% Generate the stiffness matrix for N floors
stiffness_array = zeros(1,N+1);
alt_stiffness_array = zeros(1,N);
for i=1:N-1
  stiffness_array(i) = 2*K;
  alt_stiffness_array(i) = -K;
endfor
stiffness_array(N) = k+K;
stiffness_array(N+1) = k;
alt_stiffness_array(N) = -k;

stiffness_matrix = diag(stiffness_array) + diag(alt_stiffness_array, -1) + diag(alt_stiffness_array, +1);
stiffness_matrix

% Generate the damping matrix
damping_array = zeros(1,N+1);
alt_damping_array = zeros(1,N);
for i=1:N-1
  damping_array(i) = 2*L;
  alt_damping_array(i) = -L;
endfor
damping_array(N) = L+l;
damping_array(N+1) = l;
alt_damping_array(N) = -l;

damping_matrix = diag(damping_array) + diag(alt_damping_array, -1) + diag(alt_damping_array, +1);
damping_matrix

% Formulate first order state equation matricies:

I = eye(N+1);
first_order_M = [I,zeros(N+1);zeros(N+1), mass_matrix]
first_order_D = [zeros(N+1), -1*I; stiffness_matrix, damping_matrix]

A = first_order_M\first_order_D

[V, D] = eig(A)

initial_conditions = [0.2;0;0;0;0;0;0;0];
a = V\initial_conditions
%{
syms t;

lamda_array = transpose(diag(D))
_A_array = sym(zeros(1,2*(N+1)))
for i=1:2*(N+1)
  _A_array(i) = exp(-1*lamda_array(i)*t);
endfor
_A_ = diag(_A_array)

y(t) = V*_A_*a;
test = double(y(0.1));
test(1);

figure(1);


% Calculate time domain response to initial conditions
datapoints1 = [];
datapoints2 = [];
datapoints3 = [];
for time = 0:0.02:3
  time
  displ_ = real(double(subs(y,(vpa(time)))));
  datapoints1 = [datapoints1 displ_(1)];
  datapoints2 = [datapoints2 displ_(2)];
  datapoints3 = [datapoints3 displ_(3)];
end
hold on
time = 0:0.02:3;
plot(time,(datapoints1),'-');
plot(time,(datapoints2),'-');
plot(time,(datapoints3),'-');
%}

function y = zero_force_response(lamda_matrix, V, a, t)
  N = length(a)/2;
  lamda_array = transpose(diag(lamda_matrix));
  _A_array = zeros(1,2*(N));
  for i=1:2*(N)
  _A_array(i) = exp(-1*lamda_array(i)*t);
  endfor
  _A_ = diag(_A_array);
  y = V*_A_*a;
endfunction

figure(1);


% Calculate time domain response to initial conditions
datapoints1 = [];
datapoints2 = [];
datapoints3 = [];
for time = 0:0.001:20
  displ_ = real(zero_force_response(D, V, a, time));
  datapoints1 = [datapoints1 displ_(1)];
  datapoints2 = [datapoints2 displ_(2)];
  datapoints3 = [datapoints3 displ_(3)];
end
hold on
time = 0:0.001:20;
plot(time,(datapoints1),'-');
plot(time,(datapoints2),'-');
plot(time,(datapoints3),'-');
legend


figure(2);
hold off

function amp = damped_forced_vibration(D,M,F,w)
  Y0 = (D+M*sqrt(-1)*w)\F;
  for j=1:length(F)/2
    amp(j) = sqrt(Y0(j)*conj(Y0(j)));
    %phase(j) = log(conj(Y0(j))/Y0(j))/(2*sqrt(-1));
  endfor
endfunction



% Calculate frequency response functions
plottype=3;

all_disp = [];
for w = 1:130;
  displ = damped_forced_vibration(first_order_D, first_order_M, [10;0;0;0;10;0;0;0], w);
  all_disp = [all_disp displ(4)];
end
w = 1:130;


% Log plot
if (plottype == 2)
  semilogy((w./(2*pi)),abs(all_disp),'-');

% Linear plot
elseif (plottype == 3)
  plot((w./(2*pi)),(all_disp),'-');
end
figure(3);


L = 2000;
Fs = 1000;
T = 1/Fs;
time = (0:L-1)*T;


for i=1:5
  hold on
  test = fft(datapoints1((i*2000-1999):(i*2000)));
  P2 = abs(test/L);
  P1 = P2(1:L/2+1);
  P1(2:end-1) = 2*P1(2:end-1);
  f = Fs*(0:(L/2))/L;
  plot(f(1:20), P1(1:20))
endfor


[pks, locs] = findpeaks(all_disp);
min_peaks = []
if (length(locs) > 3)
  for i=1:(length(locs) - 1)
    [pks2, locs2] = findpeaks(all_disp(locs(i):locs(i+1)), 'DoubleSided');
    min_peaks(i) = locs2
  endfor
  natural_frequencies = w(locs(1)+min_peaks(1)-1)/(2*pi)
endif

w(locs)
natural_frequencies = w(locs)/(2*pi)
w(2)/(2*pi)
w(40)/(2*pi)
[pks3, locs3] = findpeaks(all_disp(2:40));
[maximum, index] = max(pks3)

maximum

compute_pendulum_length(w(locs))




