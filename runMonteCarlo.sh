#!/bin/bash

# Copy all *.m files to the "USER_EXPERIMENT" directory
cp ~/Git/RobotariumRepositories/robotarium_job_executor/user_data/*.m ~/Git/RobotariumRepositories/MonteCarlo/USER_EXPERIMENT/

# Run the simulation
matlab -nodesktop -r "RunSims; exit"
