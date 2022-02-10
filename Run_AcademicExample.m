% Simulate controllers whose parameters are stored in cntrDesign_1.mat/cntrDesign_2.mat
%
% Simulates gain scheduling controllers without and with anti-windup 
% Simulates linear controllers applied individually to nonlinear planty
% Fig. 1 contains initial convergence phase of observer
% Fig. 2 is the figure used in the paper
%
% Author: Klaus Kefferpütz, University of Applied Sciences Augsburg 2021
% email: klaus.kefferpuetz@hs-augsburg.de 
clc
clear all
close all

% add path to folder with auxiliary functions (saturation, design script)
addpath(genpath('./tools'))

TSim    = 6.0;
tStep   = 3.5;
tOffset = 3.0;


initVal   = 1;    % initial reference commmand
refVal    = 1.5;  % command applied at tStep

load cntrDesign_1.mat
% load cntrDesign_2.mat % alternate parameter setting

% sampling time for controller
cntr.TS = 1e-3; 

umax      = 800; %  -> deactivate input constraints setting umax very large
% umax      = 8; %  simulate with input constraint |u|<umax 
 
% operating points
cntr.xAP_1 = [ 1   0]';
cntr.xAP_2 = [ 1.5 0]';

cntr.uAP_1 = -1;
cntr.uAP_2 = -1.5;

% initial state: start in operating point 1
xInit = cntr.xAP_1;

% linear system at operating point 1
system.A1 = [ 0  1;
              1  -1   ];
system.B1 = [ 0;
              1 ];
          
% linear system at operating point 2    
system.A2 = [ 0  1;
              1  -1.5   ];
system.B2 = [ 0;
              1 ];
          
% output is linear i.e. y=x_1
system.C = [1 0];          
          
% augmented system for PI state space design
system.augA1 = [ system.A1 zeros(2,1); 
                -system.C   0 ];
system.augB1 = [system.B1;
                    0 ];

system.augA2 = [ system.A2 zeros(2,1); 
                -system.C   0 ];
system.augB2 = [system.B2;
                    0 ];

cntr.umax = umax;

% simulate gain-scheduling controller without anti-windup
cntr.awOn = 0;

out = sim('gainSchedExample_nonlinPlant_nonlinCntr.slx', [0 TSim], simset('SrcWorkspace','current'));
out.ref.time    = out.ref.time-tOffset*ones(size(out.ref.time));
out.output.time = out.output.time-tOffset*ones(size(out.output.time));
out.input.time  = out.input.time-tOffset*ones(size(out.input.time));
out.sched1.time = out.sched1.time-tOffset*ones(size(out.sched1.time));
out.sched2.time = out.sched2.time-tOffset*ones(size(out.sched2.time));

% simulate gain-scheduling controller with anti-windup
cntr.awOn = 1;

outAW = sim('gainSchedExample_nonlinPlant_nonlinCntr.slx', [0 TSim], simset('SrcWorkspace','current'));
outAW.ref.time    = outAW.ref.time-tOffset*ones(size(outAW.ref.time));
outAW.output.time = outAW.output.time-tOffset*ones(size(outAW.output.time));
outAW.input.time  = outAW.input.time-tOffset*ones(size(outAW.input.time));
outAW.sched1.time = outAW.sched1.time-tOffset*ones(size(outAW.sched1.time));
outAW.sched2.time = outAW.sched2.time-tOffset*ones(size(outAW.sched2.time));

umax      = 800; % no saturation for linear controllers applied to nonlinear plant

cntr.umax = umax;
% set controller parameters for controller 1 (linear controller 1 on
% nonlinear plant
cntr.A         = cntr.A1;
cntr.B         = cntr.B1;
cntr.Ki        = cntr.Ki1;
cntr.Kp        = cntr.Kp1;
cntr.K         = cntr.K_1; 
cntr.L         = cntr.L1;
cntr.uAP       = cntr.uAP_1;
cntr.xAP       = cntr.xAP_1;
cntr.initState = xInit-cntr.xAP;

outLin_1 = sim('gainSchedExample_nonlinPlant_linearCntr.slx', [0 TSim], simset('SrcWorkspace','current'));
outLin_1.output.time = outLin_1.output.time-tOffset*ones(size(outLin_1.output.time));
outLin_1.input.time  = outLin_1.input.time-tOffset*ones(size(outLin_1.input.time));


% set controller parameters for controller 2 (linear controller 2 on
% nonlinear plant
cntr.A         = cntr.A2;
cntr.B         = cntr.B2;
cntr.Ki        = cntr.Ki2;
cntr.Kp        = cntr.Kp2;
cntr.K         = cntr.K_2; 
cntr.L         = cntr.L2;
cntr.uAP       = cntr.uAP_2;
cntr.xAP       = cntr.xAP_2;
cntr.initState = xInit-cntr.xAP;

outLin_2 = sim('gainSchedExample_nonlinPlant_linearCntr.slx', [0 TSim], simset('SrcWorkspace','current'));
outLin_2.output.time = outLin_2.output.time-tOffset*ones(size(outLin_2.output.time));
outLin_2.input.time  = outLin_2.input.time-tOffset*ones(size(outLin_2.input.time));


%%
close all

% Figures included in response letter
figure(1)
subplot(311)
hold on
plot(out.ref.time, out.ref.signals.values, 'k')
plot(outLin_1.output.time, outLin_1.output.signals.values, 'g')
plot(outLin_2.output.time, outLin_2.output.signals.values, '--g')
plot(out.output.time, out.output.signals.values, 'b')
plot(outLin_1.output.time(1:1000:end), outLin_1.output.signals.values(1:1000:end), 'xg', 'MarkerSize', 10)
plot(outLin_2.output.time(500:1000:end), outLin_2.output.signals.values(500:1000:end), 'og', 'MarkerSize', 10)
grid on
xlabel(['$t$ in $s$'],'interpreter','latex')
ylabel(['$y(t)$'],'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
xlim([-2.5 2.5])
ylim([0.8 2.0])
subplot(312)
hold on
plot(outLin_1.input.time, outLin_1.input.signals.values, 'g')
plot(outLin_2.input.time, outLin_2.input.signals.values, '--g')
plot(out.input.time, out.input.signals.values, 'b')
plot(outLin_1.input.time(1:1000:end), outLin_1.input.signals.values(1:1000:end), 'xg', 'MarkerSize', 10)
plot(outLin_2.input.time(500:1000:end), outLin_2.input.signals.values(500:1000:end), 'og', 'MarkerSize', 10)
xlabel(['$t$ in $s$'],'interpreter','latex')
ylabel(['$u(t)$'],'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([-2.5 2.5])
subplot(313)
hold on
plot(out.sched1.time, out.sched1.signals.values, '--b')
plot(out.sched2.time, out.sched2.signals.values, 'b')
xlabel(['$t$ in $s$'],'interpreter','latex')
ylabel(['$\mu_i(p(t))$'],'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([-2.5 2.5])
ylim([-0.2 1.2])


% paper Fig. 4/Fig. 5
figure(2)
subplot(311)
hold on
plot(out.ref.time, out.ref.signals.values, 'k')
plot(outLin_1.output.time, outLin_1.output.signals.values, 'g')
plot(outLin_2.output.time, outLin_2.output.signals.values, '--g')
plot(out.output.time, out.output.signals.values, 'b')
plot(outAW.output.time, outAW.output.signals.values, '--r')
plot(outLin_1.output.time(1:1000:end), outLin_1.output.signals.values(1:1000:end), 'xg', 'MarkerSize', 10)
plot(outLin_2.output.time(500:1000:end), outLin_2.output.signals.values(500:1000:end), 'og', 'MarkerSize', 10)
grid on
xlabel(['$t$ in $s$'],'interpreter','latex')
ylabel(['$y(t)$'],'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
xlim([0 2.5])
subplot(312)
hold on
plot(outLin_1.input.time, outLin_1.input.signals.values, 'g')
plot(outLin_2.input.time, outLin_2.input.signals.values, '--g')
plot(out.input.time, out.input.signals.values, 'b')
plot(outAW.input.time, outAW.input.signals.values, '--r')
plot(outLin_1.input.time(1:1000:end), outLin_1.input.signals.values(1:1000:end), 'xg', 'MarkerSize', 10)
plot(outLin_2.input.time(500:1000:end), outLin_2.input.signals.values(500:1000:end), 'og', 'MarkerSize', 10)
xlabel(['$t$ in $s$'],'interpreter','latex')
ylabel(['$u(t)$'],'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([0 2.5])
subplot(313)
hold on
plot(out.sched1.time, out.sched1.signals.values, '--b')
plot(out.sched2.time, out.sched2.signals.values, 'b')
plot(outAW.sched1.time, outAW.sched1.signals.values, '--r')
plot(outAW.sched2.time, outAW.sched2.signals.values, 'r')
xlabel(['$t$ in $s$'],'interpreter','latex')
ylabel(['$\mu_i(p(t))$'],'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
grid on
xlim([0 2.5])
ylim([-0.2 1.2])