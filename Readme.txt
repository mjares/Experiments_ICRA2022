There are two main sets of scripts to run. 

### Main Experiments ###
 This first set should be prioritized since it inlcudes the LOE abrupt vs. intermittent experiments.
 All results from these scripts will be output to the folder 'Main_Results'. Every subfolder includes
a .txt file with the expected files to be output.
 These scripts can be run in parallel since each is independent from the other, however, the following
order is recommended:
 1) Script_TestPerformance_RandomTrajectory.py
 2) Script_TestPerformance_RandomTrajectory_Abrupt.py
 3) Script_TestPerformance_RandomTrajectory_Intermittent.py

### Secondary Experiments ###
 This second set should be run only if there is time after the main experiments are done, since it 
 inlcudes the low level controller comparison experiments.
 All results from these scripts will be output to the folder 'Secondary_Results'. Every subfolder 
includes a .txt file with the expected files to be output.
 These scripts can be run in parallel since each is independent from the other, however, the following
order is recommended:
 4) Script_TestPerformance_RandomTrajectory_WindLowLevel.py
 5) Script_TestPerformance_RandomTrajectory_WindLowLevel_C1C3.py
 6) Script_TestPerformance_RandomTrajectory_WindLowLevel_C2C3.py
 7) Script_TestPerformance_RandomTrajectory_WindLowLevel_Abrupt.py
 8) Script_TestPerformance_RandomTrajectory_WindLowLevel_AbruptC1C3.py
 9) Script_TestPerformance_RandomTrajectory_WindLowLevel_AbruptC2C3.py
 10) Script_TestPerformance_RandomTrajectory_WindLowLevel_InterC1C3.py
 11) Script_TestPerformance_RandomTrajectory_WindLowLevel_InterC2C3.py
