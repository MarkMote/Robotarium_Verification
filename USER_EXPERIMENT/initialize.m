%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize and record data 
%using the Robotarium simulator
%Paul Glotfelter 
%3/24/2016

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();
rb.actually_save_data = false;

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = N_actual;
% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_show_figure(false).set_save_data(false).build();

% Initialize x so that we don't run into problems later.  This isn't always
% necessary
x = r.get_poses();
r.step();

position_int = create_si_position_controller('XVelocityGain', 2, 'YVelocityGain', 2);
si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.16);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.75, 'AngularVelocityLimit', pi);

initial_conditions = generate_initial_conditions(N, 'Spacing', 0.1, 'Width', 1, 'Height', 0.6);
   
args = {'PositionError', 0.1, 'RotationError', 100};
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller(args{:});

start = tic;

while(true)
    
    if(toc(start) > 30)
        break
    end
    
    if(init_checker(x, initial_conditions))
        break
    end
    
    x = r.get_poses();
    dxi = position_int(x(1:2, :), initial_conditions(1:2, :));
    
    dxmax = 0.07;
    for i = 1:N
        if norm(dxi(:,i)) > dxmax
            dxi(:,i) = dxi(:,i)/norm(dxi(:,i))*dxmax;
        end
    end
    
    dxi = si_barrier_certificate(dxi, x(1:2, :));
    dxu = si_to_uni_dyn(dxi, x);
    
    %r.set_leds(1:N, repmat([0 0 255 0]', 1, N));
    r.set_velocities(1:N, dxu);
    r.step();
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
