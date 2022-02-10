% Implementation 
%
% Autor: Klaus Kefferpütz
% University of Applied Sciences Augsburg
% Datum:05.07.2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ optResult, solOpt ] = designRobustMdlRecovAWPoly( plant, designAW )
    Anom = plant.Anom;
    Bnom = plant.Bnom;
    C = plant.C;
    
    Arob = plant.Arob;
    Brob = plant.Brob;
    
    D = plant.D;
    
    sysOrd  = size(Anom,2);
    inputDim   = size(Bnom,2);
    outputDim  = size(C,1);
    
    % Define LMI variables
    Q       = sdpvar( sysOrd, sysOrd, 'symmetric' ); 
    L       = sdpvar( inputDim,sysOrd );
    U       = sdpvar( inputDim, inputDim,'diagonal');
    gammaVar    = sdpvar( 1, 1, 'full' );
    gammaU      = sdpvar( 1, 1, 'full' );      
    gammaVarRob = sdpvar( 1, 1, 'full' );
    gammaUrob   = sdpvar( 1, 1, 'full' );      
    

    % Lyapunov matrix positive definite
    LMIs = [ Q > 0 ];
    LMIs = [ LMIs,   U > 0 ];
    LMIs = [ LMIs, gammaVar >0 ];
    LMIs = [ LMIs, gammaU > 0];
    LMIs = [ LMIs, gammaVarRob >0 ];
    LMIs = [ LMIs, gammaUrob > 0];
    for ii=1:1:inputDim
        LMIs =  [ LMIs, [   Q                       L';
                            L     designAW.beta(ii)^2*designAW.u(ii)^2*eye(inputDim)] >= 0 ];
    end
 
    Cj = C;
    Dj = D;
    
    LMIs = [ LMIs, [Q*Anom'+Anom*Q+L'*Bnom'+Bnom*L      Bnom*U-L'*designAW.A_beta        zeros(sysOrd,inputDim)            Bnom                     Q*Cj'+L'*Dj';
                       (Bnom*U-L'*designAW.A_beta)'     -2*U                             designAW.A_beta'             -designAW.A_beta'                 U*Dj';
                         zeros(sysOrd,inputDim)'          designAW.A_beta               -gammaVar*eye(inputDim)     zeros(inputDim,inputDim)       zeros(inputDim,outputDim);
                               Bnom'                     -designAW.A_beta                zeros(inputDim,inputDim)     -gammaU*eye(inputDim)        zeros(inputDim,outputDim);
                          (Q*Cj'+L'*Dj')'                  (U*Dj')'                      zeros(inputDim,outputDim)'   zeros(inputDim,outputDim)'           -gammaU*eye(outputDim)] < -eye(sysOrd+3*inputDim+outputDim)*1e-6 ];
                           
      LMIs = [ LMIs, [ Q*Arob'+Arob*Q+L'*Brob'+Brob*L      Brob*U-L'*designAW.A_beta       zeros(sysOrd,inputDim)               Brob                      Q*Cj'+L'*Dj';
                           (Brob*U-L'*designAW.A_beta)'     -2*U                            designAW.A_beta'               -designAW.A_beta'                 U*Dj';
                             zeros(sysOrd,inputDim)'        designAW.A_beta                 -gammaVarRob*eye(inputDim)       zeros(inputDim,inputDim)       zeros(inputDim,outputDim);
                               Brob'                       -designAW.A_beta                  zeros(inputDim,inputDim)       -gammaUrob*eye(inputDim)        zeros(inputDim,outputDim);
                           (Q*Cj'+L'*Dj')'                  (U*Dj')'                        zeros(inputDim,outputDim)'    zeros(inputDim,outputDim)'       -gammaUrob*eye(outputDim)] < -eye(sysOrd+3*inputDim+outputDim)*1e-6 ];
                           
                           
                           
%     for ii = 1:1:outDim
%             
%             Cj=C(ii,:);
%             Dj=D(ii,:);
%             
%             LMIs = [ LMIs, gammaVar(ii)>0 ];
%             
%             LMIs = [ LMIs, [Q*Anom'+Anom*Q+L'*Bnom'+Bnom*L         Bnom*U-L'*designAW.A_beta        zeros(SysOrd,inDim)         Q*Cj'+L'*Dj';
%                                (Bnom*U-L'*designAW.A_beta)'         -2*U                 designAW.A_beta'             U*Dj';
%                                zeros(SysOrd,inDim)'           designAW.A_beta       -gammaVar(ii)*eye(inDim)    zeros(inDim,1);
%                                (Q*Cj'+L'*Dj')'                  (U*Dj')'               zeros(inDim,1)'         -gammaVar(ii)*eye(1)] < 0 ];
%                            
%             LMIs = [ LMIs, [Q*Arob'+Arob*Q+L'*Brob'+Brob*L         Brob*U-L'*designAW.A_beta        zeros(SysOrd,inDim)         Q*Cj'+L'*Dj';
%                                (Brob*U-L'*designAW.A_beta)'         -2*U                 designAW.A_beta'             U*Dj';
%                                zeros(SysOrd,inDim)'           designAW.A_beta       -gammaVar(ii)*eye(inDim)    zeros(inDim,1);
%                                (Q*Cj'+L'*Dj')'                  (U*Dj')'               zeros(inDim,1)'         -gammaVar(ii)*eye(1)] < 0 ];
%        
%     end
%     
%     
    opts = sdpsettings('solver','sdpt3','sdpt3.gaptol',1e-5,'verbose',0,'sdpt3.steptol', 2e-4, 'sdpt3.stoplevel',0,'sdpt3.use_corrprim',1,...
                             'sdpt3.maxit',1200, 'sdpt3.scale_data',0, 'savesolveroutput', 1,'cachesolvers',1);
%     opts = sdpsettings('solver','sedumi','sedumi.eps',1e-5,'verbose',0,'sedumi.bigeps',1e-2,'sedumi.maxiter',1000, 'savesolveroutput', 1,'cachesolvers',1);
    %Solve feasibility problem
    solOpt = solvesdp( LMIs, designAW.eta*gammaU+(1-designAW.eta)*gammaVar, opts );
%     solOpt = solvesdp( LMIs,designAW.eta*sum(gammaVar(:))+(1-designAW.eta)*(-logdet(invpos(Q))), opts );
%     solOpt = solvesdp( LMIs,sum(gammaVar), opts );
    
    Q           = double( Q );
    L           = double( L );
    U           = double( U ); 
    gammaVar    = double( gammaVar );
    
    
    optResult.U = U;
    optResult.L = L;
    
    optResult.Q = Q;
    optResult.P = inv(Q);
    optResult.K = L/Q;
    optResult.gamma = sum(gammaVar);








end

