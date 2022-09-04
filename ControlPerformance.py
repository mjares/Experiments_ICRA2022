import numpy as np


class FullControlPerformance:
    def __init__(self):
        # Control
        self.controller = ''
        # Results
        self.mode_performance = []
        self.mode_loss = []
        self.mode_tags = []
        self.no_modes = 0
        # Details
        self.mode_detailed = []

    def append_mode(self, mode_control_performance):
        self.no_modes = self.no_modes + 1
        self.mode_detailed.append(mode_control_performance)
        self.mode_performance.append(mode_control_performance.stability)
        self.mode_loss.append(mode_control_performance.avg_loss)
        self.mode_tags.append(mode_control_performance.mode_tag)

    def plot_performance(self):
        pass


class ModeControlPerformance:
    def __init__(self):
        # General
        self.no_episodes = 0
        self.episode_list = []
        # Mode
        self.mode = ''
        self.mode_range = [0, 0]
        self.mode_tag = ''
        # Performance
        self.stability_list = []
        self.loss_list = []
        self.stability = 0.0  # Percent of stable episodes
        self.avg_loss = 0.0

    def append_episode(self, ep_control_performance):
        self.no_episodes = self.no_episodes + 1
        self.episode_list.append(ep_control_performance)
        self.stability_list.append(ep_control_performance.stable_at_goal)
        self.loss_list.append(ep_control_performance.loss)

    def calculate_performance(self):
        self.stability = np.array(self.stability_list).sum()*100/self.no_episodes
        self.avg_loss = np.array(self.loss_list).mean()
        return self.stability, self.avg_loss

    def generate_mode_tag(self):
        self.mode_tag = f'{self.mode}: [{self.mode_range[0]}, {self.mode_range[1]}]'


class EpisodeControlPerformance:
    def __init__(self):
        # Conditions
        self.mode = ''
        self.fault_mag = 0.0
        self.rotor = -1
        # Performance
        self.stable_at_goal = False
        self.loss = 0.0
        self.no_samples = 0
        self.no_out_of_bounds = 0  # Number of timesteps out of bounds
        self.fault_time = [0, 0]  # [starttime, endtime]

    def calculate_loss(self):
        c_stable = 0 if self.stable_at_goal else 1  # 0 if stable at goal, 1 otherwise.
        c_oob = self.no_out_of_bounds/self.no_samples
        self.loss = c_stable + (1 - c_stable) * c_oob
        return self.loss
