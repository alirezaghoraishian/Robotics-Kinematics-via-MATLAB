clear; close all; clc;

syms q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5 real;
syms d0 d1 l1 l2 l3 l4 l5 real;


%%% Configuration %%%
qq = [q1; q2; q3; q4; q5];
dqq = [dq1; dq2; dq3; dq4; dq5];
ddqq = [ddq1; ddq2; ddq3; ddq4; ddq5];
state = [qq; dqq];
dstate = [dqq; ddqq];


%%%%% Part A: Calculate Rho0i, dRho0i, and ddRho0i for each link. %%%%%

%%% Link 1; Kinematics %%%
H01 = simplify(Trans(3, d0) * Rot(3, q1));
rho11 = [-l1/2; 0; d1/2; 1];
rho01 = simplify(H01 * rho11);
drho01 = simplify(jacobian(rho01, state) * dstate);
ddrho01 = simplify(jacobian(drho01, state) * dstate);

%%% Link 2; Kinematics %%%
H12 = simplify(Trans(1, -l1) * Trans(3, d1) * Rot(1, q2));
H02 = simplify(H01 * H12);
rho22 = [0; l2/2; 0; 1];
rho02 = simplify(H02 * rho22);
drho02 = simplify(jacobian(rho02, state) * dstate);
ddrho02 = simplify(jacobian(drho02, state) * dstate);

%%% Link 3; Kinematics %%%
H23 = simplify(Trans(2, q3));
H03 = simplify(H02 * H23);
rho33 = [0; -l3/2; 0; 1];
rho03 = simplify(H03 * rho33);
drho03 = simplify(jacobian(rho03, state) * dstate);
ddrho03 = simplify(jacobian(drho03, state) * dstate);

%%% Link 4; Kinematics %%%
H34 = simplify(Rot(1, q4));
H04 = simplify(H03 * H34);
rho44 = [0; l4/2; 0; 1];
rho04 = simplify(H04 * rho44);
drho04 = simplify(jacobian(rho04, state) * dstate);
ddrho04 = simplify(jacobian(drho04, state) * dstate);

%%% Link 5; Kinematics %%%
H45 = simplify(Trans(2, l4+l5) * Rot(2, q5));
H05 = simplify(H04 * H45);
rho55 = [0; -l5/2; 0; 1];
rho05 = simplify(H05 * rho55);
drho05 = simplify(jacobian(rho05, state) * dstate);
ddrho05 = simplify(jacobian(drho05, state) * dstate);

%%% End Effector; Kinematics %%%
H0e = H05;
d0e = rho05;
dd0e = drho05;
ddd0e = ddrho05;


%%%%% Part B: Calculate Omegai0 and Alphai0 for each link. %%%%%

%%% Link 1; Kinematics %%%
omega10 = dq1 * [0; 0; 1];
alpha10 = simplify(jacobian(omega10, state) * dstate);

%%% Link 2; Kinematics %%%
R21 = simplify(H12(1:3, 1:3))';
omega21 = dq2 * [1; 0; 0];
omega20 = simplify(omega21 + R21 * omega10);
alpha20 = simplify(jacobian(omega20, state) * dstate);

%%% Link 3; Kinematics %%%
R32 = simplify(H23(1:3, 1:3))';
omega32 = dq3 * [0; 0; 0];
omega30 = simplify(omega32 + R32 * omega20);
alpha30 = simplify(jacobian(omega30, state) * dstate);

%%% Link 4; Kinematics %%%
R43 = simplify(H34(1:3, 1:3))';
omega43 = dq4 * [1; 0; 0];
omega40 = simplify(omega43 + R43 * omega30);
alpha40 = simplify(jacobian(omega40, state) * dstate);

%%% Link 5; Kinematics %%%
R54 = simplify(H45(1:3, 1:3))';
omega54 = dq5 * [0; 1; 0];
omega50 = simplify(omega54 + R54 * omega40);
alpha50 = simplify(jacobian(omega50, state) * dstate);

%%% End Effector; Kinematics %%%
omegae0 = omega50;
alphae0 = alpha50;


%%%%% Part C: Calculate Jacobian Ji for each link and Je for the end effector. %%%%%

%%% Link 1; Kinematics %%%
j1v = jacobian(drho01(1:3), dstate);
j1omega = jacobian(omega10, dstate);
j1 = [j1v(:, 1:5); j1omega(:, 1:5)];

%%% Link 2; Kinematics %%%
j2v = jacobian(drho02(1:3), dstate);
j2omega = jacobian(omega20, dstate);
j2 = [j2v(:, 1:5); j2omega(:, 1:5)];

%%% Link 3; Kinematics %%%
j3v = jacobian(drho03(1:3), dstate);
j3omega = jacobian(omega30, dstate);
j3 = [j3v(:, 1:5); j3omega(:, 1:5)];

%%% Link 4; Kinematics %%%
j4v = jacobian(drho04(1:3), dstate);
j4omega = jacobian(omega40, dstate);
j4 = [j4v(:, 1:5); j4omega(:, 1:5)];

%%% Link 5; Kinematics %%%
j5v = jacobian(drho05(1:3), dstate);
j5omega = jacobian(omega50, dstate);
j5 = [j5v(:, 1:5); j5omega(:, 1:5)];

%%% End Effector; Kinematics %%%
jev = j5v;
jeomega = j5omega;
je = j5;


%%%%% Part D: Plot Rho0e, v0e, a0e, Omegae0, Alphae0. %%%%%

%%% Configuration %%%
t = 0:0.1:10;

d0d = 0.6;
d1d = 0.1;
l1d = 0.25;
l2d = 0.3;
l3d = 0.3;
l4d = 0.1;
l5d = 0.05;

q1d = zeros(1, length(t));
q2d = zeros(1, length(t));
q3d = zeros(1, length(t));
q4d = zeros(1, length(t));
q5d = zeros(1, length(t));

dq1d = zeros(1, length(t));
dq2d = zeros(1, length(t));
dq3d = zeros(1, length(t));
dq4d = zeros(1, length(t));
dq5d = zeros(1, length(t));

ddq1d = zeros(1, length(t));
ddq2d = zeros(1, length(t));
ddq3d = zeros(1, length(t));
ddq4d = zeros(1, length(t));
ddq5d = zeros(1, length(t));
    
for i=1:length(t)

    if t(i) <= 5
        q1d(1, i) = pi/8 * (1 - cos(pi/5 * t(i)));
        dq1d(1, i) = pi/8 * pi/5 * sin(pi/5 * t(i));
        ddq1d(1, i) = pi/8 * pi/5 * pi/5 * cos(pi/5 * t(i));
    elseif t(i) > 5
        q1d(1, i) = pi/4;
        dq1d(1, i) = 0;
        ddq1d(1, i) = 0;
    end

    if t(i) <= 2
        q3d(1, i) = 30 + 5 * (1 - cos(pi/2 * t(i)));
        dq3d(1, i) = 5 * pi/2 * sin(pi/2 * t(i));
        ddq3d(1, i) = 5 * pi/2 * pi/2 * cos(pi/2 * t(i));
    elseif t(i) > 2
        q3d(1, i) = 40;
        dq3d(1, i) = 0;
        ddq3d(1, i) = 0;
    end

    if t(i) <= 3
        q4d(1, i) = pi/4 * (1 - cos(pi/3 * t(i)));
        dq4d(1, i) = pi/4 * pi/3 * sin(pi/3 * t(i));
        ddq4d(1, i) = pi/4 * pi/3 * pi/3 * cos(pi/3 * t(i));
    elseif t(i) > 3
        q4d(1, i) = pi/2;
        dq4d(1, i) = 0;
        ddq4d(1, i) = 0;
    end

    q5d(1, i) = 2 * q4d(1, i);
    dq5d(1, i) = 2 * dq4d(1, i);
    ddq5d(1, i) = 2 * ddq4d(1, i);

end


%%% End Effector; Kinematics %%%
d0ed = subs(d0e, {d0, d1, l1, l2, l3, l4, l5, q1, q2, q3, q4, q5}, {d0d, d1d, l1d, l2d, l3d, l4d, l5d, q1d, q2d, q3d, q4d, q5d});
dd0ed = subs(dd0e, {d0, d1, l1, l2, l3, l4, l5, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5}, {d0d, d1d, l1d, l2d, l3d, l4d, l5d, q1d, q2d, q3d, q4d, q5d, dq1d, dq2d, dq3d, dq4d, dq5d});
ddd0ed = subs(ddd0e, {d0, d1, l1, l2, l3, l4, l5, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5, ddq1, ddq2, ddq3, ddq4, ddq5}, {d0d, d1d, l1d, l2d, l3d, l4d, l5d, q1d, q2d, q3d, q4d, q5d, dq1d, dq2d, dq3d, dq4d, dq5d, ddq1d, ddq2d, ddq3d, ddq4d, ddq5d});
omegae0d = subs(omegae0, {d0, d1, l1, l2, l3, l4, l5, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5, ddq1, ddq2, ddq3, ddq4, ddq5}, {d0d, d1d, l1d, l2d, l3d, l4d, l5d, q1d, q2d, q3d, q4d, q5d, dq1d, dq2d, dq3d, dq4d, dq5d, ddq1d, ddq2d, ddq3d, ddq4d, ddq5d});
alphae0d = subs(alphae0, {d0, d1, l1, l2, l3, l4, l5, q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5, ddq1, ddq2, ddq3, ddq4, ddq5}, {d0d, d1d, l1d, l2d, l3d, l4d, l5d, q1d, q2d, q3d, q4d, q5d, dq1d, dq2d, dq3d, dq4d, dq5d, ddq1d, ddq2d, ddq3d, ddq4d, ddq5d});

figure(1);
plot(t, d0ed(1, :), '-r', t, d0ed(2, :),  '--k', t, d0ed(3, :), '-.b')
xlabel('time (s)');
ylabel('\rho_e^0 (rad)');

figure(2);
plot(t, dd0ed(1, :), '-r', t, dd0ed(2, :),  '--k', t, dd0ed(3, :), '-.b')
xlabel('time (s)');
ylabel("\rho_e^0' (rad)");

figure(3);
plot(t, ddd0ed(1, :), '-r', t, ddd0ed(2, :),  '--k', t, ddd0ed(3, :), '-.b')
xlabel('time (s)');
ylabel("\rho_e^0'' (rad)");

figure(4);
plot(t, omegae0d(1, :), '-r', t, omegae0d(2, :),  '--k', t, omegae0d(3, :), '-.b')
xlabel('time (s)');
ylabel("\omega_0^e (rad)");

figure(5);
plot(t, alphae0d(1, :), '-r', t, alphae0d(2, :),  '--k', t, alphae0d(3, :), '-.b')
xlabel('time (s)');
ylabel("\alpha_0^e (rad)");

grid on;







