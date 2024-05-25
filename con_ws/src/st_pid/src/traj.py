import numpy as np

class TrajectoryProcess:
    def __init__(self, refpose):
        self.refpose = refpose
        self.min_distance_index = 0
        self.k_st = 1.5
        self.k_p = 1
        self.k_i = 1
        self.k_d = 0.05
        self.k_p_b = 1
        self.k_i_b = 1
        self.k_d_b = 0.05

    def calculate_min_distance(self, currentpose):
        distances = np.linalg.norm(self.refpose - currentpose, axis=1)
        self.min_distance_index = np.argmin(distances)
        print("min_distance", np.amin(distances))
        return np.amin(distances)

    def calculate_headings(self):
        ref_headings = []
        for i in range(len(self.refpose) - 1):
            heading = np.arctan2(self.refpose[i+1, 1] - self.refpose[i, 1], self.refpose[i+1, 0] - self.refpose[i, 0])
            ref_headings.append(heading)
        # Optionally, set the last heading to be the same as the second last
        ref_headings.append(ref_headings[-1])  # Assume last heading continues in same direction
        return np.array(ref_headings)

    def normalize_angle(self, angle):
        angle = np.degrees(angle)
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return np.radians(angle)

    def process_poses_stanley(self, currentpose, current_heading, v):
        cross_track_error = self.calculate_min_distance(currentpose)
        ref_headings = self.calculate_headings()

        # Ensure the min_distance_index is within the bounds of ref_headings
        if self.min_distance_index >= len(ref_headings):
            self.min_distance_index = len(ref_headings) - 1

        ref_heading = ref_headings[self.min_distance_index]
        ref_heading = self.normalize_angle(ref_heading)

        yaw_cross_track = np.arctan2(currentpose[1] - self.refpose[self.min_distance_index+1][1], 
                                currentpose[0] - self.refpose[self.min_distance_index+1][0])
        yaw_path2ct = ref_heading - yaw_cross_track
        print("yaw_cross_track", np.rad2deg(yaw_cross_track))
        yaw_path2ct = -self.normalize_angle(yaw_path2ct)
        
        if yaw_path2ct > 0:
            cross_track_error = abs(cross_track_error)
        else:
            cross_track_error = -abs(cross_track_error)
        print("cross_track_error", cross_track_error)

        # Heading error calculation
        heading_error = ref_heading - current_heading
        heading_error = -self.normalize_angle(heading_error)
        print("heading_error", np.rad2deg(heading_error))

        # Steering command calculation
        steercmd = heading_error + np.arctan2((self.k_st * cross_track_error), (v + 0.00001))
        steercmd = self.normalize_angle(steercmd)
        steercmd = steercmd / np.pi

        print(f"Current Pose: {currentpose}, Ref Pose : {self.refpose[self.min_distance_index]} Ref Heading: {np.rad2deg(ref_heading)}")
        print(f"Cross Track Error: {cross_track_error}, Heading Error: {np.rad2deg(heading_error)}")
        print(f"Steering Command: {steercmd}")
        print(f"Current Heading: {np.rad2deg(current_heading)}")
        return steercmd

    def pidthrottle(self, v_error, sample_time):
        desired_accel = (self.k_p * v_error) + (self.k_i * v_error * sample_time) + (self.k_d * v_error / sample_time)
        if desired_accel > 0:
            throttle = max(0, min(desired_accel, 1))
        else:
            throttle = 0
        return throttle

    def pidbrake(self, v_error, sample_time):
        desired_accel = (self.k_p_b * v_error) + (self.k_i_b * v_error * sample_time) + (self.k_d_b * v_error / sample_time)
        if desired_accel < -2:
            brake = max(0, min(abs(desired_accel), 1))
        else:
            brake = 0
        return brake
