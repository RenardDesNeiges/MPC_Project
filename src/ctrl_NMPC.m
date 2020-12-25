function [ctrl] = ctrl_NMPC(quad) 
    %%
    import casadi.*
    opti = casadi.Opti(); % Optimization problem 
    
    N = 25; % MPC horizon 
    n  = 12;
    
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
    
    opti.minimize(...
                 ( X(10,1:N) - REF(1) ) * ( X(10,1:N) - REF(1) )' + ... %minize X error
                 ( X(11,1:N) - REF(2) ) * ( X(11,1:N) - REF(2) )' + ... %minize Y error
        10  *    ( X(12,1:N) - REF(3) ) * ( X(12,1:N) - REF(3) )' + ... %minize Z error
                     ( X(6, 1:N) - REF(4) ) * ( X(6, 1:N) - REF(4) )' ) %%+ ...   %minize YAW error
%         10000 *  ( sum( sum( epsilon_u.*epsilon_u ) ))            + ...
%         1000 *   sum( sum ( epsilon_u ) ) ...
%        ) 
    
    
%         10   *   ( X(10,1:N) - REF(1) ) * ( X(10,1:N) - REF(1) )' + ... %terminal X constraint
%         10   *  ( X(11,1:N) - REF(2) ) * ( X(11,1:N) - REF(2) )' + ... %terminal Y constraint
%         10   *  ( X(12,1:N) - REF(3) ) * ( X(12,1:N) - REF(3) )' + ... %terminal Z constraint
%  10   *  ( X(6 ,1:N) - REF(4) ) * ( X(6 ,1:N) - REF(4) )'  %terminal YAW constraint

    
    % ADD EPSILON THERE
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CONSTRAINT SATISFACTION : 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for k=1:N % loop over control intervals
    
      opti.subject_to(X(:,k+1) == X(:,k) + quad.Ts*quad.f(X(:,k),U(:,k)) )
    
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
  