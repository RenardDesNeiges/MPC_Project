function [th,xh,uh,rh] = saveRunRecover(quad, ctrl )
      sim.t = 0;
      x0 = zeros(12,1); x0(4) = pi;
      sim.x = x0;
      sim.z_hat = zeros(3,1); % Offset free for z-dimension
      Ts = quad.Ts;
      Tf = 30;
      
      th = (1:ceil(Tf/Ts))./ceil(Tf/Ts);
      xh = zeros(12,ceil(Tf/Ts));
      uh = zeros(4,ceil(Tf/Ts));
      rh = zeros(4,ceil(Tf/Ts));
      
      
      ref = @(t,x) quad.MPC_ref(t, Tf);
      
      if nargin < 6, input_bias = 0; end
      
      
      fprintf('Simulating...\n');
      for i = 1:ceil(Tf/Ts)
        progress = (i/ceil(Tf/Ts))*100;
        fprintf('... %.2f %% \n', progress);
        
        % Reference (we just want regulation for this example)
        sim(i).ref = [0;0;0;0];
        
        % Simulate forward in time
        sim(i+1).t = sim(i).t + Ts;
        
        % Compute control law
        sim(i).u = ctrl(sim(i).x, sim(i).ref);

        % Compute system dynamics
        sim(i+1).x = quad.step(sim(i).x, sim(i).u + input_bias, Ts);
        
        [sim(i).omega, sim(i).theta, sim(i).vel, sim(i).pos] = ...
          quad.parse_state(sim(i).x);
        
        % Save coordinates for later plotting
        th(i) = Ts*i;
        rh(:,i) = [0;0;0;0]; 
        xh(:,i) = sim(i).x;
        uh(:,i) = sim(i).u;
      end
      sim(end) = [];
end