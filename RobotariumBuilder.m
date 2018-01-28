%% RobotariumBuilder
% This class handles the creation of the Robotarium object.  In particular,
% it controls and sets the parameters for your simulation/experiment.
%% Function Summary 
% * get_available_agents(): $\emptyset \to \mathbf{Z}^{+}$ returns the number of available agents
% (random for each instantiation)
% * set_number_of_agents(): $\mathbf{Z}^{+} \to RobotariumBuilder$ sets the number of
% agents, returning the RobotariumBuilder object
% * set_save_data(): $\{false, true\} \to RobotariumBuilder$ sets whether
% to save data for this experiment.  
% * build(): $\emptyset \to Robotarium$ builds a Robotarium object with the
% specified parameters
%% Example Usage 
%   % Example showing potential usage of the RobotariumBuilder object.
%   % Note that get_available_agents() returns the number of available
%   % agents, which is random.  If you need a particular number of agents,
%   % this should be specified in the experiment descriptor when you
%   % eventually submit your experiment to the Robotarium.  
%   % Or you can design your experiment to handle any number of agents.
%
%   % set_save_data() controls whether the Robotarium records
%   % your simulation/experimental data.
%
%   robo_builder = RobotariumBuilder() 
%   N = robo_builder.get_available_agents()
%   robo_obj =
%   robo_builder.set_number_of_agents(N).set_save_data(true).build()

classdef RobotariumBuilder < ARobotariumBuilder
    %ROBOTARIUMBUILDER This class handles creation of the Robotarium object
    %that communicates with the GRITSbots.
    %  This class is really just a helper to assist with creating the
    %  Robotarium object.  In particular, this object allows you to set
    %  properties for your simulation and eventual experiment.  Right now,
    %  these properties are: number of agents and whether to save data.
    
    % THIS CLASS SHOULD NEVER BE MODIFIED
    
    % Gets properties from abstract class as well.
    properties
        %% Custom
        static_ICs;                % Option to hold ICs constant for each experiment - brought in from Experiment class
        expNum;                    % The current experiment iteration in the MC loop
        prev_data;                 % The initial specs that are reused if static_ICs == true

        %% Default
        boundaries = [-0.6, 0.6, -0.35, 0.35]; 
        robot_diameter = 0.08
    end
    
    methods
        
        function this = RobotariumBuilder()
            %% Custom
            % Get parameters from Experiment
            ExpData = Experiment();      % Probe Experiment class to get options 
            this.static_ICs = ExpData.static_ICs;
            
            % Infer the Experiment Number (e)
            w = what;
            this.expNum = length(w.mat) + 1 ;
            if this.expNum > 1 % correct for existence of userSpec.mat file
                this.expNum = this.expNum-1;
            end
            
            % Set the prev_data variable and number of robots
            if (this.expNum > 1)
                load userSpec;                                   % Load
                this.prev_data = userSpec;                       % prev_data contains the information saved (in the Robotarium Class) after the first experiment
                this.available_agents = this.prev_data.N;        % previously used number of robots - Even if not static, need to maintain same number of robots
            else                                             % else choose a random number of robots
                this.prev_data = -1;
                this.available_agents = randi(14) + 1;
                
            end
            
          %% Default  
%           this.available_agents = randi(14) + 1; 
          this.number_of_agents = -1;
        end
        
        function number_of_agents = get_available_agents(this)
           number_of_agents = this.available_agents;
        end
        
        function robotarium_obj = build(this)
            %% Default 
            assert(this.number_of_agents > 0, 'You must set the number of agents for this experiment');
            
            arena_width = this.boundaries(2) - this.boundaries(1);
            arena_height = this.boundaries(4) - this.boundaries(3);
            
            %% Custom
            % Give option to use the same ICs every iteration
            if (this.static_ICs) && this.expNum > 1                      % Use previously saved poses if static; Currently the first experiment is always random 
                initial_poses = this.prev_data.IPs;                      % Set initial poses equal to those saved after first experiment 
            else                                                         % if first experiment or not static, generate random initial poses
                numX = floor(arena_width / this.robot_diameter);
                numY = floor(arena_height / this.robot_diameter);
                values = randperm(numX * numY, this.number_of_agents);
                
                initial_poses = zeros(3, this.number_of_agents);
                
                for i = 1:this.number_of_agents
                    [x, y] = ind2sub([numX numY], values(i));
                    x = x*this.robot_diameter - (arena_width/2);
                    y = y*this.robot_diameter - (arena_height/2);
                    initial_poses(1:2, i) = [x ; y];
                end
                
                initial_poses(3, :) = rand(1, this.number_of_agents)*2*pi;
            end
            
            robotarium_obj = Robotarium(this.number_of_agents, this.save_data, this.show_figure, initial_poses);

        end
    end  
end

