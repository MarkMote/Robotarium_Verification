% Runs the Monte Carlo
clear; clc; close all
cd '/home/mark/Dropbox/MATLAB/Monte_Carlo/Robotarium_Verification'
% Note - must go into Robotartium object and make sure data is being saved
% for each experiement regardless of user specification



%% Initiate JSON file in case timeout occurs 
out.valid = false; 
out.safe  = false; 
out.message = '\nExperiment crashed';
out.simulations = 0; 
out.user_params.N = []; 
out.user_params.iterations = [];
out.details.barriers_used = [];
out.details.in_testbed = [];
out.details.safety_score = [];
out.details.ss_variance = [];

savejson('',out,'Saved_Experiments/OutData.json');

%% Initialization
maxRunTime = 20;                 % Desired time to run Monte Carlo (s)

run init
Exp = Experiment();

% Location of script
cd consensus_static
% cd examples/formation_control;

% Clear all *.mat files from the workspace
delete *.mat

%% First Run - Access Validity
% The first run is Error Free
runTime = tic;
currentTime(1) = toc(runTime);
try 
    Exp.runMain;
    time2runExp = toc(runTime);
    Exp.E = floor(maxRunTime/time2runExp);
    Exp.maxRunTime = maxRunTime;
    
    % Save user specs and check validity
    Exp.valid = Exp.interpSpecs(1, true);
catch me
    Exp.valid = false; 
    Exp.message = strcat('\nSimulation does not complete.\n\nMATLAB error message:\n- ',me.message); % If experiment runs correctly, reset message 
    Exp.E = 0;
    Exp.uSpec{1}.N = [];
    Exp.uSpec{1}.iterations = [];
    Exp.barriers_used = [];
%     cd .. 
end

%% Experiment Looping
% Additional runs include added noise

if Exp.valid % If first experiment is not Exp.valid, do not bother with the rest 
    fprintf('\n\nEstimated Runtime: %d seconds\n\n',maxRunTime);
%     fprintf('Experiment 1')
    h = waitbar(0,'Running Simulations...');
    waitbar(1/Exp.E)
    for e = 2:Exp.E
        clearvars -except Exp runTime currentTime e Exp.valid h
        currentTime(e) = toc(runTime); %#ok<SAGROW>
        timeDiff = currentTime(e)-currentTime(e-1);     % time between this and last experiment
        if timeDiff<1                                   % There must be at least one second in between experiments due to the way data is named - % #ADD more elegant solution to this problem
            pause(1.1-timeDiff)                         % Make sure it has enough time to save everything
            currentTime(e) = toc(runTime);              %#ok<SAGROW> % Save time of experiment completion
        end
        
        Exp.runMain;     % Run the experiment
%         fprintf('\nExperiment %d%\n',e);
        waitbar(e/Exp.E)
        pause(0.0001)
        if toc(runTime)> Exp.maxRunTime*1.25 % Break the loop if the maximum time is exceeded
            break
        end
        
    end
    close(h)
    
    %% Save Experiments - Could also done in "runMain" function
    Exp.saveExps()
    simRunTime = toc(runTime);
    
    %% Plot
    if Exp.make_plot
        Exp.plotSims()
    end
    
    %% Wrap it up
    cd ../Saved_Experiments
    save('Experiment','Exp')                   % #ADD unique filename
    cd ../
    clear runtime
    % close all #TEMP
    clc
    
    % Plot Results to Window
    [ currentTime' , Exp.Sval' ]  %#ok<NOPTS>
    
    % Determine whether all experiments stay in testbed
    if sum(1-Exp.inBounds)== 0
        Exp.inTestbed = true;
    else
        Exp.inTestbed = false;
    end
    
    %% Print to terminal
    
    fprintf('\n___________________________________________________\n')
    if Exp.SafetyScore>0 && Exp.inTestbed
        fprintf('\n###Evaluation: SAFE\n')
        Exp.message = strcat('\nExperiment Safe!',Exp.message);
        Exp.safe = true;
        if mean(Exp.Sval)-2*var(Exp.Sval) > 0
            fprintf('###Confidence: High\n')
        elseif mean(Exp.Sval)-var(Exp.Sval) > 0
            fprintf('###Confidence: Medium\n')
        else
            fprintf('###Confidence: Low\n')
        end
        
    else
        fprintf('\n###NOT SAFE\n')
        fprintf('###Reason(s): ')
        Exp.message = strcat('\nExperiment Unsafe\nReason(s):\n',Exp.message); 
        Exp.safe = false; 
        if Exp.SafetyScore<0
            fprintf('###- Too many collisions\n')
            Exp.message = strcat(Exp.message,'- Too many collisions\n');
        end
        if ~Exp.inTestbed
            fprintf('###- Robots leave testbed\n')
            Exp.message = strcat(Exp.message,'- Robots leave testbed\n');
        end
    end
    fprintf('\n###Experiment Info:')
    fprintf('\n###Specifications_Valid: %u',Exp.valid)
    fprintf('\n###Barriers_Used: %u', Exp.barriers_used) 
    fprintf('\n###Stays in the Testbed: %u',Exp.inTestbed)
    fprintf('\n###Number of Robots: %u',Exp.uSpec{1}.N)
    fprintf('\n###Experiment Length: %u seconds',ceil(Exp.uSpec{1}.iterations/33))
    fprintf('\n###Experiments Executed: %u ',Exp.E)
    fprintf('\n###Experiment_Safety_Value: %.2f',Exp.SafetyScore)
    fprintf('\n###Variance: %.2f', var(Exp.Sval))
    fprintf('\n###Time_Normalized_Experiment_Safety_Value: %.2f',Exp.SafetyScore_TD)
    fprintf('\n___________________________________________________\n\n')

%% Create CSVs
% Experiment Info
M = [Exp.uSpec{1}.N , ceil(Exp.uSpec{1}.iterations/33) , Exp.E, Exp.SafetyScore];
csvwrite('Saved_Experiments/ExperimentInfo',M )

else % not valid 
    cd ../

    fprintf('\n___________________________________________________\n\n')

    Exp.message = strcat('\nExperiment Invalid\n\nReason(s):\n',Exp.message); 
    
end

%% Print Message and Save to JSON file 

fprintf('\nMessage to user:\n')
fprintf(Exp.message);
fprintf('\n\n___________________________________________________\n\n');
csvwrite('Saved_Experiments/user_message',Exp.message)

% Make JSON file 
out.valid = Exp.valid; 
out.safe  = Exp.safe; 
out.message = Exp.message;
out.simulations = Exp.E; 
out.user_params.N = Exp.uSpec{1}.N; 
out.user_params.iterations = Exp.uSpec{1}.iterations;
out.details.barriers_used = Exp.barriers_used;
out.details.in_testbed = Exp.inTestbed;
out.details.safety_score = Exp.SafetyScore;
out.details.ss_variance = var(Exp.Sval);

savejson('',out,'Saved_Experiments/OutData.json')

