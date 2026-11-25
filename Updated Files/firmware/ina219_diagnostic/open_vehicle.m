% Open the separate vehicle model for manual deployment. Does not upload.
taskDir=fileparts(mfilename('fullpath'));
addpath(taskDir,fullfile(fileparts(taskDir),'motor_imu'));
load_system(fullfile(taskDir,'Vehicle_Drive.slx'));
data=get_param('Vehicle_Drive','CoderTargetData');
data.Runtime.BuildAction='Build, load and run';
set_param('Vehicle_Drive','CoderTargetData',data);
open_system('Vehicle_Drive');
fprintf('Keep motor battery disconnected and BEC off during deployment.\n');
fprintf('Deploy using Hardware > Run on board > Deploy > Build, Deploy & Start.\n');
