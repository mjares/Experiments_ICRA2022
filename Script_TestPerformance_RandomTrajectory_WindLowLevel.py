import os
import sys
from pathlib import Path
import pickle

import numpy as np
from numpy.random import default_rng
import matplotlib.pyplot as plt

# Adding local libs to path
parentFolder = Path(os.getcwd()).parent.parent.parent
sys.path.append(str(parentFolder) + '\\Z. Local mods')

# Local libs
import quadcopter
import Controller
from Helpers_Generic import set_random_square_path, plot_3d_trajectory, plot_features, LowLvlPID, \
    generate_intermittent_values
from ControlPerformance import FullControlPerformance, ModeControlPerformance, EpisodeControlPerformance

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

### Inits ###
rng = default_rng()
no_episodes = 1000
starttime_low = 0
starttime_high = 1
endtime = 31000

verbose = True
show = False
# Low Lvl Ctrl
C1 = LowLvlPID.LOE.value
C2 = LowLvlPID.AtN.value
# Supervisory Controller Types
controller_list = ['UniformC1C2', 'C1', 'C2']
# Operating Modes
#####
## *** Plot to check if wind shows a changepoint
operating_modes = [{'PosNoise': [0.01, 2]}, {'PosNoise': [2, 4]}, {'PosNoise': [4, 6]}, {'PosNoise': [6, 8]},
                   {'Wind': [0.01, 5]}, {'Wind': [5, 10]}, {'Wind': [10, 15]}, {'Wind': [15, 20]}]
for controller in controller_list:
    print(f'Controller: {controller}')
    # Filename
    saveFilename = f'Secondary_Results/Nominal/Results_{controller}_Nominal_WndPsN_EpCount_{no_episodes}.rs'
    # Performance
    Results = FullControlPerformance()
    Results.controller = controller

    ### Trajectory tracking ###
    for op_mode in operating_modes:
        modeResults = ModeControlPerformance()
        modeResults.mode = list(op_mode.keys())[0]
        modeResults.mode_range = op_mode[modeResults.mode]
        modeResults.generate_mode_tag()

        for ii in range(no_episodes):
            starttime = rng.integers(starttime_low, starttime_high)
            if verbose:
                print('Episode: ', ii + 1)
            total_steps = []
            trajectories = []

            # Make objects for quadcopter
            QUADCOPTER = {
                str(1): {'position': [0, 0, 0], 'orientation': [0, 0, 0], 'L': 0.3, 'r': 0.1, 'prop_size': [10, 4.5],
                         'weight': 1.2}}
            quad = quadcopter.Quadcopter(QUADCOPTER)
            Quads = {str(1): QUADCOPTER[str(1)]}

            # Create blended controller and link it to quadcopter object
            BLENDED_CONTROLLER_PARAMETERS = {
                'Motor_limits': [0, 9000],
                'Tilt_limits': [-10, 10],
                'Yaw_Control_Limits': [-900, 900],
                'Z_XY_offset': 500,
                'Linear_PID': {'P': [300, 300, 7000], 'I': [0.04, 0.04, 4.5], 'D': [450, 450, 5000]},
                'Linear_To_Angular_Scaler': [1, 1, 0],
                'Yaw_Rate_Scaler': 0.18,
                'Angular_PID': C1,
                'Angular_PID2': C2,
                 }

            ctrl = Controller.Blended_PID_Controller(quad.get_state,
                                                     quad.set_motor_speeds, quad.get_motor_speeds,
                                                     quad.stepQuad, quad.set_motor_faults, quad.setWind, quad.setNormalWind,
                                                     params=BLENDED_CONTROLLER_PARAMETERS)
            if controller == 'C3':
                pass
            elif controller[:7] == 'Uniform':
                ctrl.set_controller('Uniform')
            else:
                ctrl.set_controller(controller)
            goals, safe_region, total_waypoints = set_random_square_path()
            currentWaypoint = 0
            ctrl.update_target(goals[currentWaypoint], safe_region[currentWaypoint])

            # Operating Mode
            no_modes = len(op_mode)
            modes_list = sorted(op_mode)  # Sorting the key list guarantees consistency
            rng = default_rng()
            rand_mode = rng.integers(0, no_modes)
            rand_key = modes_list[rand_mode]
            ctrl.set_fault_mode(rand_key)
            fault_type = rand_key
            # Setting up selected mode
            if rand_key == 'Nominal':
                fault_mag = 0
                rotor = -1
                # Verbose
                if verbose:
                    print('Operating Mode: ', rand_key)  # No changes needed for nominal conditions
            elif rand_key == 'Rotor':
                # Randomly generating anomaly magnitude
                rand_coeff = rng.uniform()  # **** Consider including ranges
                # Getting loe range from operating modes dict
                mode_range = op_mode[rand_key]
                fault_mag = (mode_range[1] - mode_range[0]) * rand_coeff + mode_range[0]
                # Setting LOE values on a single rotor
                faults = [0, 0, 0, 0]
                rotor = rng.integers(0, 4)  # Random Rotor Selection
                faults[rotor] = fault_mag
                ctrl.set_motor_fault(faults)
                ctrl.set_fault_time(starttime, endtime)
                # Verbose
                if verbose:
                    print('LOE (%): ', fault_mag)
            elif rand_key == 'AttNoise':
                rotor = -1
                # Randomly generating anomaly magnitude
                rand_coeff = rng.uniform()  # **** Consider including ranges
                # Getting attitude noise range from operating modes dict
                mode_range = op_mode[rand_key]
                fault_mag = (mode_range[1] - mode_range[0]) * rand_coeff + mode_range[0]
                ctrl.set_attitude_sensor_noise(fault_mag)
                ctrl.set_fault_time(starttime, endtime)
                # Verbose
                if verbose:
                    print('Attitude Noise (rad): ', fault_mag)
            elif rand_key == 'Wind':
                rotor = -1
                # Randomly generating wind magnitude
                rand_coeff = rng.uniform()  # **** Consider including ranges
                # Getting position noise range from operating modes dict
                mode_range = op_mode[rand_key]
                fault_mag = (mode_range[1] - mode_range[0]) * rand_coeff + mode_range[0]
                direction = rng.integers(0, 4)
                # magnitude = 4
                if direction == 0:
                    winds = [-fault_mag, 0, 0]
                elif direction == 1:
                    winds = [fault_mag, 0, 0]
                elif direction == 2:
                    winds = [0, -fault_mag, 0]
                elif direction == 3:
                    winds = [0, fault_mag, 0]
                ctrl.set_steady_wind_magnitude(winds)
                ctrl.set_fault_time(starttime, endtime)
                # Verbose
                if verbose:
                    print('Wind (): ', fault_mag)
            elif rand_key == 'PosNoise':
                rotor = -1
                # Randomly generating anomaly magnitude
                rand_coeff = rng.uniform()  # **** Consider including ranges
                # Getting attitude noise range from operating modes dict
                mode_range = op_mode[rand_key]
                fault_mag = (mode_range[1] - mode_range[0]) * rand_coeff + mode_range[0]
                ctrl.set_position_sensor_noise(fault_mag)
                ctrl.set_fault_time(starttime, endtime)
                # Verbose
                if verbose:
                    print('Position Noise (m): ', fault_mag)
            elif rand_key == 'Intermittent_Rotor':
                duration = op_mode[rand_key][0]
                period_range = op_mode[rand_key][1]
                starttime, endtime, switch_period = generate_intermittent_values([starttime_low, starttime_high],
                                                                                 duration,
                                                                                 period_range)
                fault_mag = switch_period
                # Rotor
                faults = [0, 0, 0, 0]
                rotor = rng.integers(0, 4)  # Random Rotor Selection
                faults[rotor] = 1
                ctrl.set_intermittent_motor_fault(faults, switch_period)
                ctrl.set_fault_time(starttime, endtime)
                if verbose:
                    print(f'Intermittent Rotor: {switch_period} [{starttime}, {endtime}]')
            elif rand_key == 'Temp_Rotor':
                ctrl.set_fault_mode('Rotor')
                duration_range = op_mode[rand_key]
                duration = rng.integers(duration_range[0], duration_range[1])
                endtime = starttime + duration
                fault_mag = duration
                # Rotor
                faults = [0, 0, 0, 0]
                rotor = rng.integers(0, 4)  # Random Rotor Selection
                faults[rotor] = 1
                ctrl.set_motor_fault(faults)
                ctrl.set_fault_time(starttime, endtime)
                if verbose:
                    print(f'Temporary Full Rotor: {duration} [{starttime}, {endtime}]')
            # Trajectory Tracking Initialization
            done = False
            stableDone = False
            stepcount = 0
            out_of_bounds = 0
            stableAtGoal = 0
            obs_log = []
            weight_log = []
            # First control step
            obs, weight = ctrl.update(return_weight=True)  # No action needed for Random Uniform Control.

            while not done:
                stepcount += 1

                obs, weight = ctrl.update(return_weight=True)
                # Logs
                weight_log.append(weight)
                obs_log.append(obs)

                if ctrl.check_safety_bound():
                    out_of_bounds = out_of_bounds + 1
                if stepcount > 10000:  # Max stepcount reached
                    print('Not Stable at Goal')
                    done = True
                    trajectories = ctrl.get_trajectory()
                    total_steps.append(ctrl.get_total_steps())

                if ctrl.is_at_pos(goals[currentWaypoint]):  # If the controller has reached the waypoint
                    if currentWaypoint < total_waypoints - 1:  # If not final goal
                        currentWaypoint += 1  # Next waypoint
                        ctrl.update_target(goals[currentWaypoint], safe_region[currentWaypoint-1])
                    else:  # If final goal
                        stableAtGoal += 1  # Number of timesteps spent within final goal
                        if stableAtGoal > 100:
                            if verbose:
                                print('Stable at Goal')
                            done = True
                            stableDone = True
                            trajectories = ctrl.get_trajectory()
                            total_steps.append(ctrl.get_total_steps())
                else:
                    stableAtGoal = 0

            ### Plotting
            trajectories = np.array(trajectories)
            if show:
                plot_3d_trajectory(trajectories, safe_region, fault_type, show=False)
                plt.figure()
                plt.plot(weight_log)
                plt.figure()
                plt.plot(ctrl.loe_log)
                plot_features(np.array(obs_log), starttime)

            ## To Performance results
            epResults = EpisodeControlPerformance()
            epResults.mode = rand_key
            epResults.fault_mag = fault_mag
            epResults.rotor = rotor
            epResults.stable_at_goal = stableDone
            epResults.no_samples = stepcount
            epResults.no_out_of_bounds = out_of_bounds
            epResults.calculate_loss()
            epResults.fault_time = [starttime, endtime]

            modeResults.append_episode(epResults)

        if show:
            plt.show()

        modeResults.calculate_performance()
        Results.append_mode(modeResults)

    datasetFile = open(saveFilename, 'wb')  # Creating file to write
    pickle.dump(Results, datasetFile)
    datasetFile.close()
    print('\n')

