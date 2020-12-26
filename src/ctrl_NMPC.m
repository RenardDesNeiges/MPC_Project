function [ctrl] = ctrl_NMPC(quad, pA, zA, yA, uA) 
    %%
    import casadi.*
    opti = casadi.Opti(); % Optimization problem 
    
    Ts = quad.Ts; % sampling period
    
    N = 40; % MPC horizon 
    n  = 12;% state dimensionality
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % DECISION VARIABLES : 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X = opti.variable(12,N+1); % state trajectory variables     
    U = opti.variable(4, N); % control trajectory (throttle, brake)
    X0 = opti.parameter(12,1); % initial state
    REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % COST FUNCTION AND MINIMIZATION TARGET : 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     epsilon_u = opti.variable(4,N); % slack variable

    
    if nargin < 2
        pA = 1; zA = 100; yA = 1; uA = 0.1;
    end
%%
    opti.minimize(...
        pA *     ( X(10,1:N) - REF(1) ) * ( X(10,1:N) - REF(1) )' + ...     % minimize X error
        pA *     ( X(11,1:N) - REF(2) ) * ( X(11,1:N) - REF(2) )' + ...     % minimize Y error
        zA *     ( X(12,1:N) - REF(3) ) * ( X(12,1:N) - REF(3) )' + ...    % minimize Z error
        yA *     ( X(6, 1:N) - REF(4) ) * ( X(6, 1:N) - REF(4) )' + ...     % minimize YAW error
        uA *     ( sum(U,1) * sum(U,1)' ) )                                 % minimize total throttle use
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CONSTRAINT SATISFACTION : 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for k=1:N % loop over control intervals
        
      % Runge-Kutta 4 integration
      
      k1 = quad.f( X(:,k)              , U(:,k) );
      k2 = quad.f( X(:,k) + (Ts/2) * k1, U(:,k) );
      k3 = quad.f( X(:,k) + (Ts/2) * k2, U(:,k) );
      k4 = quad.f( X(:,k) +  Ts    * k3, U(:,k) );
    
      opti.subject_to(X(:,k+1) == X(:,k) + ( (Ts/6) * (k1 + 2*k2 + 2*k3 + k4) ) );
    
    end
    
    for i = 1: size(U,1)
        opti.subject_to( 0 <= U(i,:));
        opti.subject_to( U(i,:)  <= 1.5 );
    end
    
    opti.subject_to(X(:,1)==X0);
    
    ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
    
    %%
    
end

function u = eval_ctrl(x, ref, opti, X0, REF, X, U) 
    % −−−− Set the initial state and reference −−−− 
    opti.set_value(X0, x);
    opti.set_value(REF, ref);
    
    % −−−− Setup solver NLP −−−−−−
    ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false); 
    opti.solver('ipopt', ops);
    
    % −−−− Solve the optimization problem −−−−
    sol = opti.solve();
    assert(sol.stats.success == 1, 'Error computing optimal input');
    
    u = opti.value(U(:,1));
    
    % Use the current solution to speed up the next optimization
    opti.set_initial(sol.value_variables());
    opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
  