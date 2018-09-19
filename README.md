# Monte Carlo Verification of Robobotarium Experiments

_This package contains the tools for accessing the safety and robustness of experiments submitted for use in the Robotarium_

**Author:** Mark Mote
___
### Using the Verifier:
The verifier runs a Monte Carlo simulation on the user submitted experiments in the _USER_EXPERIMENT_ directory, and outputs a *.json file containing information on the safety, robustness, and validity of the experiment. 

##### Setup:
Before attempting to run the main script __RunSims.m__ you must first change the directory at the top of this script to reflect the current location of the folder on your machine. 

##### Executing from MATLAB: 
The _main_ script for running the verification test is __RunSims.m__. This script will execute on the script contained in the _USER_EXPERIMENT_ directory. 

##### Executing from terminal:
The Monte Carlo may also be executed directly from the terminal using the script __runMonteCarlo .sh__. This script first copies the MATLAB files into the _USER_EXPERIMENT_ directory, then executes __RunSims.m__. The bash script will likely need to be modified to include the correct directory that you are pulling the  experiments from. 

##### Specifying Options: 
You may specify a number of paramters for the Monte Carlo. First, in the _Initialization_ section of __RunSims.m__ we set the desired length of time for which the Monte Carlo will run,
+ __desiredRunTime__ [double] (s): A lower bound on the maximum time to run the Monte Carlo - i.e. this is a cutoff time: no additional experiments will be run after this amount of time has passed. 

The rest of the options are in __Experiment.m__, 
+ __make_plot__ [true | false]: Whether to plot the robot trajectories at the end of the experiment.
+ __static_ICs__ [true | false]: If true this will use the same initial conditions for all experiments, otherwise, the poses are randomly initialized 
+ __sim_collisions__ [true | false]: Whether collisions are simulated. While simulating collision effects is more physically accurate, this slows down the execution time.
+ __damageThreshold__ [double]: A scaling factor for how much _damage_ the robots can take before the experiment is rejected. 
+ __valid_N_range__ [double:double]: The number of robots available for use in the Robotarium. If the user specifies a number outside of this range, the experiment is rejected. 
+ __valid_iter_range__ [Int]: The number of iterations that the user is allowed to specify for the experiment. Note that there are approximately 33 iterations/second of experiment time. 

___
### Output File 
The experiment results are both printed to the terminal, and stored in a _json_ file __Saved_Experiments/OutData.json__ containing the following fields: 
+ __valid__ [true | false]: True if the user has specified a valid number of robots and iterations.
+ __safe__ [true | false}: True if there is not an exessive amount of collisions observed in simulation, and all robots remain inside the testbed bounds for each experiment. 
    + Disclaimer: Safety assessment of black box multi-agent systems is a notoriously difficult problem. The results of this may vary, particularly if a low number of total simulations is conducted in the Monte Carlo. This should however, be able to weed out _very unsafe_ algorithms (e.g. consensus) while passing through _very safe_ algorithms (e.g. when barrier functions are used) 
    + A _confidence factor_ can be derived by looking at the variance of the safety values amoung the conducted experiments (included in _details.ss_variance_). 
+ __message__ [string]: A message describing the result of the experiment, and potentially diagnosing the reason for rejection. 
+ __simulations__ [Int]: The number of simulations executed in the Monte Carlo. 
+ __user_params.N__ [Int]: Number of agents specified by the user. 
+ __user_params.iterations__ [Int]: Number of iterations specified by the user (experiment length). 
+ __details.barriers_used__ [true | false]: Whether or not barrier functions were called by the user. 
+ __details.in_testbed__ [true | false]: Whether or not all robots remained inside of the arena for every expeiment. 
+ __details.safety_score__ [double]: A value measuring the inferred _safety_ of the experiment.
    + This ranges from [-Inf,100] with positive values corresponding to "safe" experiments, and negative values implying "unsafe" experiments. A value of 100 means that no collisions were observed. 
    + This value is the mean of individual safety scores for each experiment. 
+ __details.ss_variance__ [double]: The variance of the individual safety scores for each expeiment.


___
### Digging Deeper into the Code: 
**Modifications to the oiginal simulator's scripts** are indicated with the comment tag #MOD. The modifications are as follows: 
- Each original class is modified 
- A few lines in the Barrier certificate utility function is modified

**Two Main scripts are added:** 
+ __RunSims__ is the main script for running the monte carlo
+ __Experiment__ class handles simulations   

**The following tags are used in the comments:**
+ #MOD: Lines/sections modified in the code
+ #REQ: Required settings in the original
+ #ADD: Requests addition to code 
+ #FIX: Indicates something not right about code

