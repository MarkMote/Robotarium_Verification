classdef Experiment < handle
    % Experiment manages the Monte Carlo Simulations
    
    properties (GetAccess = public, SetAccess = public)
        E = 1;                  % Number of experiments to run
        uSpec;                  % User specifications 
        statesLog;              % History of state data 
        Sval = [];              % Used for calculating safety score      
        SafetyScore = [];       % Metric for damage done via collisions 
        SafetyScore_TD = [];    % Time normalized safety score
        inBounds = false;       % Whether a given experiment stays in testbed 
        maxRunTime              % The maximum run time of the Monte Carlo (seconds)
        safe  = false;          % Initialize safety to false 
        valid = false;          % Initialize validity to false 
        message = '\n';         % Message returned to user 
        barriers_used = false;  % Switched to "true" iff barriers are called
        inTestbed = false;      % Whether all experiments stay in testbed
        
        % Options
        make_plot = true;       % Plot experiments (together) at end of program
        static_ICs = true;      % Keep initial pose the same for every experiment
        sim_collisions = false; % Simulate collisions?        
    end
    
    properties (GetAccess = public, SetAccess = private) % #ADD: Should be able to get some of these properties automatically by instantiating robotarium classes        
        boundaries =  [-1.5, 1.5, -1.5, 1.5];                                % Testbed boundaries
        boundary_points = {[-1.5, 1.5, 1.5, -1.5], [-1.5, -1.5, 1.5, 1.5]};  % Same
        robot_diameter = 0.08;                                               % Diameter of Robot
        
        % Options
        valid_iter_range = 32:ceil(10.*(60/0.033));              % The range of acceptable experiment lengths
        valid_N_range = 1:50;                                    % The range of robots available 
        damageThreshold = 0.03 ;                                 % A normalization factor on how much damage the robots can take
        
    end
    
    methods(Static)
        function runMain()
            % Running the main script from within this function ensures
            % the user does not have access to any "outside" data
            run main
        end
    end
    
    methods
        function this = Experiment()
            
        end
        
        function valid = interpSpecs(this,e,check)
            % Saves the user specified parameters and Initial conditions
            % Makes sure all values or acceptable
            % Should typically only run on first iteration (e=1)
            load userSpec   % Note: if needed, entire "r" object can be saved
            this.uSpec{e} = userSpec;
            valid = true;
            if check
                % Checks that all user params are valid - #ADD: there is a lot that can be added to make this function more powerful
                if ~ismember(this.uSpec{e}.N, this.valid_N_range)
                    valid = false;
                    fprintf('\n\n###EXPERIMENT STOPPED - Check # agents\n\n')
                    this.message = strcat(this.message,'- Invalid number of robots specified\n');
                end
                if ~ismember(this.uSpec{e}.iterations, this.valid_iter_range)
                    valid = false;
                    fprintf('\n\n###EXPERIMENT STOPPED - Check # iterations\n\n')
                    this.message = strcat(this.message,'- Invalid number of iterations specified\n');
                end
               load ExpData_1.mat
                if exitsArena
                    valid = false;
                    fprintf('\n\n###EXPERIMENT STOPPED - Make sure robots stay in the arena\n\n') 
                    this.message = strcat(this.message,'- Robots leave testbed\n');
                end
            end
        end
        
        
        function saveExps(this) % Save all experiments and process data (called once after all data has been generated) 
            files = what;
            uData = files.mat;
            i = 1;
            j = 1;
            for e = 1:length(uData)
                if ~strcmp(uData{e},'userSpec.mat')
                    load(uData{e})
                    if strcmp(uData{e}(1:7),'ExpData')
                        this.inBounds(i) = ~exitsArena; 
                        this.Sval(i) = Svalue;
                        i = i+1;
                    else
                        load(uData{e})
                        this.statesLog(:,:,j) = robotarium_data;
                        j = j+1;
                    end
                end
            end
            
            % Time Dependent Safety Score (normalized to 1800 iterations =
            % 1 minute)
            this.SafetyScore_TD = ...
                (1-(1800/this.uSpec{1}.iterations)*(mean(this.Sval)...
                /(this.uSpec{1}.N*this.damageThreshold)))*100;
           
            % Individial Safety Scores
            this.Sval = (1-this.Sval/(this.uSpec{1}.N*this.damageThreshold))*100;
            
            % Time Independent Safety Score
            this.SafetyScore = mean(this.Sval);
                              
            delete *.mat % clear junk from folder
        end
        
        
        function plotSims(this)
            % Load Params
            X = this.statesLog;
            completedExps = length(X(1,1,:)) ; 
            T = this.uSpec{1}.iterations;
            N = this.uSpec{1}.N;
            a = zeros(N,this.E);
            
            % Setup Plot
            % numRobots = N;
            offset = 0.05;
            
            % Scale factor (max. value of single Gaussian)
            scaleFactor = 0.5;
            figPhi = figure;
            % figure_handle = figPhi;
            
            % Plot Robotarium boundaries
            patch('XData', this.boundary_points{1}, 'YData', this.boundary_points{2}, ...
                'FaceColor', 'none', ...
                'LineWidth', 3, ...
                'EdgeColor', [0, 0.74, 0.95]);
            
            %plot(im)
            set(figPhi,'color','white','menubar','none');
            
            % Set axis
            robotPlaneAxes = gca;
            
            % Limit view to xMin/xMax/yMin/yMax
            axis(robotPlaneAxes,[this.boundaries(1) - offset,this.boundaries(2)+offset,this.boundaries(3)-offset,this.boundaries(4)+offset])
            caxis([0,1.5*scaleFactor])
            set(robotPlaneAxes,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1])
            
            % Store axes
            axis(robotPlaneAxes,'off')
            
            % Other Prep
            set(robotPlaneAxes,'position',[0 0 1 1],'units','normalized','YDir','normal')
            
            hold on
            for t = 1:10:T
                for n = 1:N
                    for e = 1:completedExps
                        if e==1  % Error free exp
                            a(n,e) = plot(X(5*n-4,t,e),X(5*n-3,t,e),'ro');
                        else     % The rest of the exps 
                            a(n,e) = plot(X(5*n-4,t,e),X(5*n-3,t,e),'bo');
                        end
                    end
                end
                pause(0.001)
                if ~(t==T)
                    delete(a)
                end
            end
        end
        
    end
    
end

