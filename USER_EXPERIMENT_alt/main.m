%% Simulator Skeleton File - Project 1 

% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder
close all

%% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder(); % Do not modify

% Get the number of available agents from the Robotarium.  
N =1;  % Do not modify (we will only be using one robot for this project)

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(true).build(); % Do not modify

% Select the number of iterations for the experiment.
iterations = 2000; 

% Other important variables
target1 = [-0.5; 0; 0];
target2 = [0.5; 0; 0];
k = 1;
isParked=false;

% Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
   x = r.get_poses();
%%%%%%%%%%%%%%%%%%%%%%%% Place Algorithm Here %%%%%%%%%%%%%%%%%%%%%%%%%
%  % Create a barrier certificate for use with the above parameters 
%    args = {'PositionError', 0.01, 'RotationError', 0.1};
%    init_checker = create_is_initialized(args{:});
%    while(~init_checker(x, target1) && isParked == false)
%     
%       unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', 0.06, ... 
%          'ProjectionDistance', 0.03);
% 
%      args = {'PositionError', 0.01, 'RotationError', 0.01};
%      automatic_parker = create_automatic_parking_controller(args{:});
% 
%      dxu = automatic_parker(x, target1);
%      dxu = unicycle_barrier_certificate(dxu, target1);      
% 
%      r.set_velocities(1:N, dxu);  
%    
%      r.step(); 
%      x = r.get_poses();
%   end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
dxu = [-0.003;0];

isParked=true;   
% dxu = [1,pi]';

%% Send velocities to agents
% dxu is the input, u, a 2x1 vector for one robot 
r.set_velocities(1:N, dxu);
   
% Send the previously set velocities to the agents.  This function must be called!
r.step(); 
end  

% Call r.call_at_scripts_end() after our experiment is over!
r.call_at_scripts_end(); % Do not modify