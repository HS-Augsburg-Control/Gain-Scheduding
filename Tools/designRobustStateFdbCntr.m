% Implementation of a convex optimization problem to design state feedback
% controllers minizing the output energy and assuring a predefined region
% of eigenvalues for both the nominal system as well as a deviated system
% 
%
% Author: Klaus Kefferpütz  University of Applied Sciences Augsburg
% Datum: 05.07.2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ K, solOpt, Q, W ] = designRobustStateFdbCntr(plant, design) 

%plant parameters
Anom      = plant.Anom;
Bnom      = plant.Bnom;
C         = plant.C;
Arob      = plant.Arob;
Brob      = plant.Brob;
C         = plant.C;

% design parameter
lambdaMin = design.lambdaMin;
lambdaMax = design.lambdaMax;
delta     = design.delta;
SysOrd    = size(Anom,2);
inDim     = size(Bnom,2);
outDim    = size(C,1);

% create LMI Variables
Q     = sdpvar( SysOrd, SysOrd, 'symmetric' );
W     = sdpvar( inDim,SysOrd);
gamma = sdpvar( outDim, 1, 'full' );

sigmaNom = Anom*Q-Bnom*W;
sigmaRob = Arob*Q-Brob*W;

% LMIs for pole placement area and optimizing output energy
LMIs = [Q > ones(SysOrd)*0.0001];

% pole placement constraints applied to noniminal and deviated system
LMIs = [LMIs, Q*Anom'+Anom*Q-Bnom*W-W'*Bnom'  < -2*lambdaMin*Q];
LMIs = [LMIs,  Q*Anom'+Anom*Q-Bnom*W-W'*Bnom'  > -2*lambdaMax*Q];

LMIs = [LMIs, Q*Arob'+Arob*Q-Brob*W-W'*Brob'  < -2*lambdaMin*Q];
LMIs = [LMIs,  Q*Arob'+Arob*Q-Brob*W-W'*Brob'  > -2*lambdaMax*Q];

LMIs = [LMIs, [ delta*(sigmaNom'+sigmaNom)          (sigmaNom'-sigmaNom);
                     (-sigmaNom'+sigmaNom)     delta*(sigmaNom'+sigmaNom) ] < 0];

LMIs = [LMIs, [ delta*(sigmaRob'+sigmaRob)          (sigmaRob'-sigmaRob);
                     (-sigmaRob'+sigmaRob)     delta*(sigmaRob'+sigmaRob) ] < 0];

% performance LMI only evaluated for nominal system
for(ii=1:1:outDim)
    LMIs = [LMIs, [ Q*Anom'+Anom*Q-Bnom*W-W'*Bnom'      Q*C(ii,:)';
                        C(ii,:)*Q                  -gamma(ii)*eye(1)] < 0];
end
                 
% define solver and options settings                 
opts = sdpsettings('solver','sdpt3','sdpt3.gaptol',1e-5,'verbose',0,'sdpt3.steptol', 2e-5,...
    'sdpt3.maxit',200, 'sdpt3.scale_data',0, 'savesolveroutput', 1,'cachesolvers',1 ); 

       
solOpt = solvesdp( LMIs,  sum(gamma), opts );

Q = double(Q);
W = double(W);
K = W/Q;
