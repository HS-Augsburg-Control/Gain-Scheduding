% Design script for controller, observer and MRAW nets
% results are stored in structure "cntr"
%
% Author: Klaus Kefferpütz, University of Applied Sciences Augsburg 2021
% email: klaus.kefferpuetz@hs-augsburg.de 
clc
clear all
close all

% path to LMI design scripts
addpath(genpath('./tools'))

umax      = 8;
cntr.umax = umax;

% Kp chosen manually, is used for both controllers and considered in the
% parameterization of the state feedback matrix 
Kp = 18; 

% operating points
cntr.xAP_1 = [ 1 0]';
cntr.xAP_2 = [ 1.5 0]';

cntr.uAP_1 = -1;
cntr.uAP_2 = -1.5;

% initial state
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
          
% augmented system for PI state space controller design -> extend by
% integrator state
system.augA1 = [ system.A1 zeros(2,1); 
                -system.C   0 ];
system.augB1 = [system.B1;
                    0 ];

system.augA2 = [ system.A2 zeros(2,1); 
                -system.C   0 ];
system.augB2 = [system.B2;
                    0 ];

% reuqirements for pole-placement area for both controllers
design1.delta = 2;
design1.lambdaMin = 3 ;
design1.lambdaMax = 6;

design2.delta = 2;
design2.lambdaMin = 3;
design2.lambdaMax = 6;

designObs.delta = 2;
designObs.lambdaMin = 30 ;
designObs.lambdaMax = 40;

% design parameters MRAW
designAW.beta   = 80;          
designAW.eta    = 0.6;         
designAW.u      = cntr.umax; 
designAW.A_beta = 0.4;         


% design robust controllers with overlapping operating regions and
% predefined pole placement areas. 
% Nominal system is the operating point where the controller should be
% designed for. The other operating point should be robustly stabilized. 
% Similar approach carried out for observer, considering the dual system
plant.Anom = system.augA1;
plant.Bnom = system.augB1;
plant.C = [ system.C 0];
plant.Arob = system.augA2;
plant.Brob = system.augB2;

% design controller 1
[ augK1, solOpt, Q, W ] = designRobustStateFdbCntr(plant, design1) 

cntr.Kp1 = Kp;
cntr.Ki1 = -augK1(3);
cntr.K_1 = augK1(1:2)-cntr.Kp1*system.C;


plant.Anom = system.A1';
plant.Bnom = system.C';
plant.C = system.C;
plant.Arob = system.A2';
plant.Brob = system.C';

% design observer 1 - dual problem
[ L1, solOpt, Q, W ] = designRobustStateFdbCntr(plant, designObs) 
cntr.L1 = L1';
cntr.C  = system.C;

plant.Anom = system.A1;
plant.Bnom = system.B1;
plant.Arob = system.A2;
plant.Brob = system.B2;
plant.C = system.C;
plant.D = 0;

% design MRAW for controller 1
[ awRes1, solOpt ] = designRobustMdlRecovAWPoly( plant, designAW )

cntr.A1   = system.A1;
cntr.B1   = system.B1;
cntr.Kaw1 = awRes1.K;
cntr.CAW  = system.C;



plant.Anom = system.augA2;
plant.Bnom = system.augB2;
plant.C = [ system.C 0];
plant.Arob = system.augA1;
plant.Brob = system.augB1;

% design controller 2
[ augK2, solOpt, Q, W ] = designRobustStateFdbCntr(plant, design2) 

cntr.Kp2 = Kp;
cntr.Ki2 = -augK2(3);
cntr.K_2 = augK2(1:2)-cntr.Kp2*system.C;


plant.Anom = system.A2';
plant.Bnom = system.C';
plant.C = system.C;
plant.Arob = system.A1';
plant.Brob = system.C';

% design observer 2 - dual problem
[ L2, solOpt, Q, W ] = designRobustStateFdbCntr(plant, designObs) 
cntr.L2 = L2';


plant.Anom = system.A2;
plant.Bnom = system.B2;
plant.Arob = system.A1;
plant.Brob = system.B1;
plant.C = system.C;
plant.D = 0;

% design MRAW for controller 2
[ awRes2, solOpt ] = designRobustMdlRecovAWPoly( plant, designAW )

cntr.A2   = system.A2;
cntr.B2   = system.B2;
cntr.Kaw2 = awRes2.K;

