N_actual = 1;
pwd_here = pwd;
cd /home/robotarium/Git/RobotariumRepositories/robotarium_matlab_backend/;
run server_init.m;
cd(pwd_here);
run initialize.m;
run main.m;
exit