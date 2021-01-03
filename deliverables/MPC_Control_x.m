classdef MPC_Control_x < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
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
      Q = eye(n); Q(n-1, n-1) = 10; Q(n,n) = 50;
      R = eye(m);
      
      % Constraints
      % u constrainedin U = { u | Mu <= m }
      M = [1;-1]; 
      m = [0.3; 0.3];
      
      % x constrained in X = { x | Fx <= f }
      F = [0 1 0 0; 0 -1 0 0]; 
      f = [0.035; 0.035];
      
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
      
      if PLOT==1
          % Plotting of terminal set
           figure();
           plot(Xf.projection(1:2), 'b');
           title('X_f for subsystem x');
           xlabel('$\dot{\beta}$', 'interpreter', 'latex');
           ylabel('\beta');
           figure();
           plot(Xf.projection(2:3), 'b');
           title('X_f for subsystem x');
           xlabel('\beta');
           ylabel('$\dot{x}$', 'interpreter', 'latex');
           figure();
           plot(Xf.projection(3:4), 'b');
           title('X_f for subsystem x');
           xlabel('$\dot{x}$', 'interpreter', 'latex');
           ylabel('x');
      end
      
      % CONSTRAINTS AND OBJECTIVE (delta formulation)
      
      %initial step
      con = (x(:,2) == A*x(:,1) + B*u(:,1)) + (M*u(:,1) <= m);
      obj = u(:,1)'*R*u(:,1);
      
      for i = 2:N-1
          con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));
          con = con + (F*x(:,i) <= f) + (M*u(:,i) <= m);
          obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
      end
      
      con = con + (Ff*(x(:,N)-xs) <= ff);
      obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);

      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
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
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      % System
      A = mpc.A;
      B = mpc.B;
      C = mpc.C;
      
      % Constraints
      % u constrained in U = { u | Mu <= m }
      M = [1;-1]; 
      m = [0.3; 0.3];
      
      % x constrained in X = { x | Fx <= f }
      F = [0 1 0 0; 0 -1 0 0]; 
      f = [0.035; 0.035];
      
      con = [A*xs + B*us == xs,...
             C*xs == ref,...
             F*xs <= f,...
             M*us <= m];
         
      obj = us^2;
      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
