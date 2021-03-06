classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection    
    L                   % Estimator gain for disturbance rejection
  end
  
  methods
    function mpc = MPC_Control_z(sys, Ts)
      mpc = mpc@MPC_Control(sys, Ts);
      
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      N = 100; %% horizon CHANGE THIS LATER
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % SYSTEM DEFINITION :
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % system dynamics ==> x+ = Ax + Bu, A & B are discrete time models
      A = mpc.A;
      B = mpc.B;
      % optimization cost ==> we optimize for min{I(x,u)} with I(x,u) = x'Qx + u'Ru
      Q = eye(n); Q(n,n) = 50; Q(n-1,n-1) = 5;
      R = eye(m);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % CONSTRAINTS DEFINITION :
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % u in U = { u | Mu <= m }
      Mc = [1;-1]; mc = [0.3; 0.2];
      % x in X = { x | Fx <= f } ==> None
      Fc = [-eye(n);eye(n)]; fc = [inf;inf;inf;inf];

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % TERMINAL SET/COST :
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      %LQR terminal control and terminal cost
      [K,Qf,~] = dlqr(A,B,Q,R);
      K = -K; %because matlab
      Acl = A+B*K;

      % Terminal set computation
      Xf = polytope([Fc;Mc*K],[fc;mc]);
      while true
          pvXf = Xf;
          [T,t] = double(Xf);
          Xf = intersect(Xf,polytope(T*Acl,t));
          if isequal(Xf,pvXf)
              break
          end
      end
      [Ff,ff] = double(Xf);

      % Plotting of terminal set
      %Xf.projection(1:2).plot();
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % MPC CONTROLLER DEFINITION :
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      %YALMIP Constraint satisfaction
      con = (x(:,2) == A*x(:,1) + B*u(:,1)) + (Mc*u(:,1) <= mc);
      obj = u(:,1)'*R*u(:,1);
      for i = 2:N-1
        con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));
          con = con + (Fc*x(:,i) <= fc) + (Mc*u(:,i) <= mc);
          obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
      end
      con = con + (Ff*(x(:,N)-xs) <= ff);
      obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);

      % YALMIP OPTIMIZER
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));

      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position
      ref = sdpvar;            
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % CONSTRAINTS DEFINITION : 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      A = mpc.A;
      B = mpc.B;
      C = mpc.C;
      D = mpc.D;
      % Constraints
      % u in U = { u | Mu <= m }
      M = [1;-1]; 
      m = [0.3; 0.2];
      con = [A*xs + B*us == xs,...
             C*xs + D*us == ref,...
             M*us <= m];
      obj = us^2;
      
      % Computing the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ... 
          {ref, d_est}, {xs, us});
      
    end
    
    
    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
      
      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      A_bar = [];
      B_bar = [];
      C_bar = [];
      L = [];
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
