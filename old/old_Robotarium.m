%% Robotarium - MODIFIED for MCV
% A class that models your communications with the GRITSbots!
%   This class handles retrieving the poses of agents, setting their
%   velocities, iterating the simulation, and saving data.

% Settings: make sure plotting turned off, barrier functions turned off

classdef Robotarium < ARobotarium
    %Robotarium This is the Robotarium simulator object that represents
    %your communications with the GRITSbots.
    
    properties (GetAccess = private, SetAccess = private) % #MOD
        %% Custom
        % Error Parameters
                
        % Systematic Robot Errors f(n) - add variation at initialization
        delV_delVc      % f(avg wheel diameter greater than nominal/ assumed value)
        delOmg_delOmgc  % f(avg wheel diameter greater than nominal/ assumed value)
        delOmg_delVc    % Drift in omega with velocity commanded -> f(uneven wheel base, uneven wheel diameter)
        delV_delOmgc    % Drift in velocity with omega commanded -> f(uneven wheel base, uneven wheel diameter)
        
        delV_delVcL
        delV_delOmgcL
        delOmg_delVcL
        delOmg_delOmgcL
        
        %  Other Custom Params
        currentIteration = 0;          % #MOD
        expNum;                        % The current experiment iteration in the MC loop
        sim_collisions                 % whether or not to simulate collisions
        S = 0 ;                        % safety score
        D_robot                        % Actual Robot diameter (set in the experiment class) 
        W = 0;                         % whether or not the robots stay in the testbed
        
        %% Default
        previous_timestep
        checked_poses_already = false
        called_step_already = true
        x_lin_vel_coef = 0.86;
        y_lin_vel_coef = 0.81;
        ang_vel_coef = 0.46;
    end
    
    methods
        
        function this = Robotarium(number_of_agents, save_data, show_figure, initial_poses) % #MOD
            %% Default
            this = this@ARobotarium(number_of_agents, save_data, show_figure, initial_poses);
            this.previous_timestep = tic;
            
            %% Custom
            % Get the Experiment Number (e)
            w = what;
            this.expNum = length(w.mat);
            if this.expNum > 1 % correct for existence of userSpec.mat file
                this.expNum = this.expNum -1;
            end
            % Load specifications from experiment class 
            ExpData = Experiment(1);
            this.sim_collisions = ExpData.sim_collisions;
            N = number_of_agents;
            this.D_robot = 0.06;   
            
            % Systematic Robot Errors f(n)
            this.delV_delVc = 0.1;                    % Percent Diameter Variation: Davg/Dnom        % check
            this.delOmg_delOmgc = this.delV_delVc;    % Same error same source                         % check
            this.delOmg_delVc = 0.2; %10e-4;                % Orientation Drift with velocity                % check
            this.delV_delOmgc = 0.01; %10e-5;                % Velocity Drift with turn rate                  % check
           
            % Create Robot Error Matricies
            this.delV_delVcL     = 1+this.delV_delVc.*(randn(1,N));
            this.delV_delOmgcL   = 0+this.delV_delOmgc.*(randn(1,N));
            this.delOmg_delVcL   = 0+this.delOmg_delVc.*(randn(1,N));
            this.delOmg_delOmgcL = 1+this.delOmg_delOmgc.*(randn(1,N));
        end
        
        function poses = get_poses(this)
            %% Default
            assert(~this.checked_poses_already, 'Can only call get_poses() once per call of step()!');
            
            poses = this.poses;
            
            %Include delay to mimic behavior of real system
            this.previous_timestep =  tic;
            
            %Make sure it's only called once per iteration
            this.checked_poses_already = true;
            this.called_step_already = false;
        end
        
        function step(this)
            %% Default
            %Vectorize update to states
            i = 1:this.number_of_agents;
            
            % #MOD 
            if this.sim_collisions % Applies collision dynamics and calculates safety score
                this.collisionCheck();
            elseif this.number_of_agents>1 % Estimate of safety score when collisions are off
                this.calculate_S();
            end
            if this.W == 0
                this.checkBounds(); % Check whether agents stay in testbed 
            end
                
            assert(~this.called_step_already, 'Make sure you call get_poses before calling step!');
            
            % Add error to the velocities except for first experiment #MOD
            if this.expNum > 1
                for j = 1:this.number_of_agents
                    this.velocities(1,j) = this.delV_delVcL(j)*this.velocities(1,j) + this.delV_delOmgcL(j)*this.velocities(2,j);
                    this.velocities(2,j) = this.delOmg_delVcL(j)*this.velocities(1,j) + this.delOmg_delOmgcL(j)*this.velocities(2,j);
                end
            end
            total_time = this.time_step + max(0, toc(this.previous_timestep) - this.time_step);
            
            %Update velocities using unicycle dynamics #MOD
            this.poses(1, i) = this.poses(1, i) + this.x_lin_vel_coef*total_time.*this.velocities(1, i).*cos(this.poses(3, i));
            this.poses(2, i) = this.poses(2, i) + this.y_lin_vel_coef*total_time.*this.velocities(1, i).*sin(this.poses(3, i));
            this.poses(3, i) = this.poses(3, i) + this.ang_vel_coef*total_time.*this.velocities(2, i);
            
            %Ensure that we're in the right range
            this.poses(3, i) = atan2(sin(this.poses(3, i)), cos(this.poses(3, i)));
            
            %Allow getting of poses again
            this.checked_poses_already = false;
            this.called_step_already = true;
            
            this.save();
            
            if(this.show_figure)  % #REQ This should be set to false in ARobotariumBuilder
                this.draw_robots();
            end
            
        end
        
        function collisionCheck(this) %#MOD
            %COLLISIONCHECK Simulates the collision dynamics between robots
            % This function is activated when this.simCollisions is true
            
            % Initialize
            vs = this.velocities;
            vs0 = vs;                                        % initial velocity equals commanded velocity
            N = this.number_of_agents;
            s = 0; 
            % Determine which robots are in contact with each other
            
            %states = [this.poses;this.velocities];
            Ax = this.poses(1,:)'*ones(1,N);
            Dx = (Ax) - (Ax');                               % x dist b/w robots i,j
            Ay = this.poses(2,:)'*ones(1,N);   %
            Dy = (Ay) - (Ay');                               % y dist b/w robots i,j
            D = (Dx.^2 + Dy.^2).^(0.5);                      % distance b/w robots i,j
            D_Overlap = (this.D_robot - D);            % overlap distance b/w robots i,j
            colMat = (1-eye(N)).*(D_Overlap>0); % colMat(i,j)=1 if i,j are overlapping
            colSet = cell(N,1);                 % Set of neighbors an agent is in collision with
            
            % Apply collision dynamics to all robots in contact
            for i = 1:N
                colSet{i} = find(colMat(i,:)==1);            % Set of robots in contact
                if sign(vs(1,i)) == 1                        % Determine if robot is driving forward or backwards
                    thetai = this.poses(3,i);               % so that theta i can be defined
                else
                    thetai = this.poses(3,i) - pi;
                end
                while thetai<0                               % Set range between 0 and 2pi
                    thetai = thetai+2*pi;
                end
                cti = cos(thetai);
                sti = sin(thetai);
                if ~isempty(colSet{i}) % i.e. IF robot i is in collision state
                    iter = 0;
                    for j = colSet{i}  % i is the
                        if sign(vs(1,j))==1 % Set thetaj
                            thetaj = this.poses(3,j);
                        else
                            thetaj = this.poses(3,j) - pi;
                        end
                        while thetaj<0 % Set range between 0 and 2pi
                            thetaj = thetaj+2*pi;
                        end
                        thetaij = atan2(Dy(j,i),Dx(j,i));
                        while thetaij<0 % Set range between 0 and 2pi
                            thetaij = thetaij+2*pi;
                        end
                        thetaDif = thetai-thetaij; % Angle b/w agent i and the line connecting the two agents
                        thetaDifj= thetaj-thetaij;
                        if thetaDif>pi % set range [-pi , pi]
                            thetaDif = 2*pi - thetaDif;
                        elseif thetaDif<-pi
                            thetaDif = -2*pi - thetaDif;
                        end
                        if  norm(thetaDif)<(pi/2) % the velocity of agent i is toward agent j
                            if iter == 0 
                                vs(1,i) = 0;
                                iter = 1;
                            end
                            % Change velocity and angular velocity vectors
                            % of both robots to respond to the collision
                            turn = 15*vs0(1,i)*(abs(sin(thetaDif)))^.5;
                            vs(2,j) = vs(2,j) + turn;
                            vs(1,i) = -0.25*norm(D_Overlap(i,j)^.25)...
                                *sign(vs0(1,i))*(norm(vs0(1,i)))^.5;
                            
                            % Increment Damage 
                            Vrel(i) = vs0(1,i).*cos(thetaDif) - vs0(1,j).*cos(thetaDifj);
                            % Damage term for the current experiment
                            s = s + Vrel(i).^2 +10e-4;
                            % this.SLog(e,this.currentIter+1) = this.SLog(e,this.currentIter)+s ;
                            
                        end
                    end
                end
                % Wall check
                if this.poses(1,i) - this.D_robot/2 < this.boundaries(1) && cti < 0
                    this.poses(1,i) = this.boundaries(1) + this.D_robot/2;
                    vs(1,i) = vs(1,i)*.25;
                elseif this.poses(1,i) + this.D_robot/2 > this.boundaries(2) && cti > 0
                    this.poses(1,i) = this.boundaries(2) - this.D_robot/2;
                    vs(1,i) = vs(1,i)*.25;
                end
                if this.poses(2,i) - this.D_robot/2 < this.boundaries(3) && sti < 0
                    this.poses(2,i) = this.boundaries(3) + this.D_robot/2;
                    vs(1,i) = vs(1,i)*.25;
                elseif this.poses(2,i) + this.D_robot/2 > this.boundaries(4) && sti > 0
                    this.poses(2,i) = this.boundaries(4) - this.D_robot/2;
                    vs(1,i) = -vs(1,i)*.25;
                end
            end
            this.velocities = vs;
            this.S = s + this.S;

        end
        
        function calculate_S(this) % #MOD 
            % Function to calculate safety function "S" from
            %   the states log WHEN COLLISION DYNAMICS ARE TURNED OFF
            % Method: S = net sum of the overlap distances between all
            %   robots in each iteration of each experiment
            Ax = this.poses(1,:)'*ones(1,this.number_of_agents);
            Dx = triu(Ax) - triu(Ax');
            Ay = this.poses(2,:)'*ones(1,this.number_of_agents);
            Dy = triu(Ay) - triu(Ay');
            D = (Dx.^2 + Dy.^2).^(0.5);
            % Get overlap distance and take sum, excluding negative values
            % for which there no overlap
            D_Overlap = triu((2*this.D_robot/2 - D),1);
            s =  sum((D_Overlap(D_Overlap>0)),1);
            this.S = s/10 + this.S;
            fprintf('%f\n',this.S)
            1
        end
        
        function checkBounds(this) 
            if max(abs(this.poses(1,:))) > this.boundaries(2) - this.robot_diameter/2
                this.W = 1; 
            elseif max(abs(this.poses(2,:))) > this.boundaries(4) - this.robot_diameter/2
                this.W = 1; 
            end
        end
        
        function call_at_scripts_end(this) %#MOD
            %% Save Robotarium data and accociated experiment number  #FIX: Combine the state data and the Sval into a single object - also combine user data? -
            if true % (this.save_data) #MOD
                this.mat_file_path.robotarium_data = this.mat_file_path.robotarium_data(:, 1:(this.current_saved_iterations-1));
                Svalue = this.S;
                exitsArena = this.W; 
                save(strcat('ExpData_', num2str(this.expNum)), 'Svalue', 'exitsArena')
            end
            
            %% Custom
            % Save user specs % #MOD #ADD: initial conditions - add this.setICs function with possible rand argument - no
            if this.expNum == 1
                userSpec.N = this.number_of_agents;
                userSpec.iterations = this.current_saved_iterations-1;
                for i = 1:userSpec.N
                    % save poses
                    % j = 3*(i-1)+1;
                    k = 5*(i-1)+1;
                    stateData = this.mat_file_path.robotarium_data(:, 1:(this.current_saved_iterations-1));
                    userSpec.IPs(1:3,i) = stateData(k:(k+2), 1);
                end
                save('userSpec','userSpec') % Note: overwrites file each experiment! #ADD time and date data like with "robotarium_data"
            end
            
        end
        
    end
end
