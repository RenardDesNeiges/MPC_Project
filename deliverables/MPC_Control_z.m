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
      
      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      Ts = 0.2;
      Tset = 8;
      N = Tset/Ts;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system
      global PLOT;
      
      % System
      A = mpc.A;
      B = mpc.B;
      Q = eye(n); 
      Q(n,n) = 10;
      R = eye(m);
      
      % Constraints
      % u constrained in U = { u | Mu <= m }
      M = [1;-1]; 
      m = [0.3; 0.2];
      
      % x consztrained in X = { x | Fx <= f }
      F = [-eye(2); eye(2)]; 
      f = [inf; inf; inf; inf];
      
      % Compute LQR controller for unconstrained system
      [K,Qf,~] = dlqr(A,B,Q,R);
      
      % MATLAB defines K as -K, so invert its signal
      K = -K; 
      
      % Compute maximal invariant set
      Xf = polytope([F;M*K],[f;m]);
      Acl = [A+B*K];
      
      while 1
          prevXf = Xf;
          [T,t] = double(Xf);
          preXf = polytope(T*Acl,t);
          Xf = intersect(Xf, preXf);
          if isequal(prevXf, Xf)
              break
          end
      end
      [Ff,ff] = double(Xf);
      
      if PLOT == 1
          % Plotting of terminal set
           figure;
           plot(Xf, 'r');
           title('X_f for subsystem z');
           xlabel('$\dot{z}$', 'interpreter', 'latex');
           ylabel('z');
      end
      
      % CONSTRAINTS AND OBJECTIVE (delta formulation)
      
      % initial step
      con = (x(:,2) == A*x(:,1) + B*u(:,1) + B*d_est) + (M*u(:,1) <= m);
      obj = u(:,1)'*R*u(:,1);
      
      for i = 2:N-1
          con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i) + B*d_est);
          con = con + (F*x(:,i) <= f) + (M*u(:,i) <= m);
          obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
      end
      
      con = con + (Ff*(x(:,N)-xs) <= ff);
      obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);
      
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
      %   d_est  - disturbance estimate
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.3)
      ref = sdpvar;
            
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      % System
      A = mpc.A;
      B = mpc.B;
      C = mpc.C;
      D = mpc.D;
      
      % Constraints
      % u constrained in U = { u | Mu <= m }
      M = [1;-1]; 
      m = [0.3; 0.2];
      con = [A*xs + B*(us+d_est) == xs,...
             C*xs + D*us == ref,...
             M*us <= m];
      obj = us^2;
      

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
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
      
      A_bar = [mpc.A, mpc.B; zeros(1,size(mpc.A,2)), 1];
      B_bar = [mpc.B; zeros(size(mpc.B,2))];
      C_bar = [mpc.C, zeros(size(mpc.C,1))];
      p = [0.05 0.075 0.15]; % hand-tuned poles
      L = -place(A_bar', C_bar', p).'; % design of gain L to satisfy pole 
                                       % placement
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
