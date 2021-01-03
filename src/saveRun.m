function [th,xh,uh,rh] = saveRun(quad, ctrl_x, ctrl_y, ctrl_z, ctrl_yaw, input_bias)
      sim.t = 0;
      x0 = zeros(12,1); x0(4) = pi
      sim.x = x0;
      sim.z_hat = zeros(3,1); % Offset free for z-dimension
      Ts = quad.Ts;
      Tf = 30;
      
      th = (1:ceil(Tf/Ts))./ceil(Tf/Ts);
      xh = zeros(12,ceil(Tf/Ts));
      uh = zeros(4,ceil(Tf/Ts));
      rh = zeros(4,ceil(Tf/Ts));
      
      if nargin >= 3
        ctrl = quad.merge_controllers(ctrl_x, ctrl_y, ctrl_z, ctrl_yaw);
      else
        ctrl = ctrl_x;
        ctrl_z.L = [];
      end
      ref = @(t,x) quad.MPC_ref(t, Tf);
      
      if nargin < 6, input_bias = 0; end
      
      for i = 1:ceil(Tf/Ts)
        
        progress = (i/ceil(Tf/Ts))*100
        
        % Compute reference
        sim(i).ref = ref(sim(i).t, sim(i).x);
        rh(:,i) = [0;0;0;0]; %sim(i).ref;
        
        % Simulate forward in time
        sim(i+1).t = sim(i).t + Ts;
        
        % Compute control law
        if ~isempty(ctrl_z.L) % Doing offset free in the z-dimension
          
          % Compute control input for z-dimension
          z_input = ctrl_z.get_u(sim(i).z_hat, sim(i).ref(3));
          
          % Run the estimator
          sim(i+1).z_hat = ctrl_z.A_bar * sim(i).z_hat + ...
            ctrl_z.B_bar * z_input ...
            + ctrl_z.L * (ctrl_z.C_bar * sim(i).z_hat - sim(i).x(quad.ind.pos(3)));
          
          % Compute control input
          sim(i).u = ctrl(sim(i).x, sim(i).ref, sim(i).z_hat);
          
        else
          sim(i).u = ctrl(sim(i).x, sim(i).ref);
        end
        
        sim(i+1).x = quad.step(sim(i).x, sim(i).u + input_bias, Ts);
        
        [sim(i).omega, sim(i).theta, sim(i).vel, sim(i).pos] = ...
          quad.parse_state(sim(i).x);
      
        th(i) = Ts*i;
        xh(:,i) = sim(i).x;
        uh(:,i) = sim(i).u;
      end
      sim(end) = [];
end