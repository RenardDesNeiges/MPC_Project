classdef MPC_Control_yaw < MPC_Control
  
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
      
      N = 20; %% horizon CHANGE THIS LATER
      
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
      Q = eye(n); Q(n,n) = 2;
      R = eye(m);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % CONSTRAINTS DEFINITION :
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % u in U = { u | Mu <= m }
      Mc = [1;-1]; mc = [0.2; 0.2];
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
%       figure();
%       plot(Xf.projection(1:2), 'b');
%       title('X_f for subsystem x');
%       xlabel('$\dot{\gamma}$', 'interpreter', 'latex');
%       ylabel('\gamma');
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % MPC CONTROLLER DEFINITION :
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      %YALMIP Constraint satisfaction
      con = (x(:,2) == A*x(:,1) + B*u(:,1)) + (Mc*u(:,1) <= mc);
      obj = u(:,1)'*R*u(:,1);
      for i = 2:N-1
        con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));
        con = con + (Fc*x(:,i) <= fc) + (Mc*u(:,i) <= mc);
        obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
      end
      con = con + (Ff*x(:,N) <= ff);
      obj = obj + x(:,N)'*Qf*x(:,N);

      % YALMIP OPTIMIZER
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
      
      % Reference position
      ref = sdpvar;            
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % CONSTRAINTS DEFINITION : 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      A = mpc.A;
      B = mpc.B;
      C = mpc.C;
      % Constraints
      % u in U = { u | Mu <= m }
      M = [1;-1]; 
      m = [0.2; 0.2];
      con = [A*xs + B*us == xs,...
             C*xs == ref,...
             M*us <= m];
      obj = us^2;

      
      % Computing the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ...
          ref, {xs, us});
      
    end
  end
end
