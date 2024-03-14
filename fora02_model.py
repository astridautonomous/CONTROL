import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from traj import TrajectoryProcess

refpose123=np.linspace([0,0],[10000,10000],10001)

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
        self.motor_lookup_rpm = [929.7, 1156.19, 1451.5, 1485.38, 1550, 1625, 1735, 1881, 1938, 2023, 2250, 2375, 2439, 2500, 2600, 2750, 3000, 3200, 3400, 3500, 3700, 3800, 3875, 3950, 4057, 4270, 4287, 4296, 5362.88]
        self.motor_lookup_tork = [41.164, 34.44, 28.24, 26.256, 23.304, 20.008, 18.204, 16.4, 15.088, 13.94, 12.464, 11.316, 10.004, 9.02, 7.708, 6.888, 5.904, 5.084, 4.264, 3.608, 2.788, 2.296, 1.804, 1.476, 1.148, 0.984, 0.954, 0.82, 0.2]
        self.xc = 50
        self.yc = 0
        self.theta = 0
        self.delta = 0
        
        self.L = 1.5


        # Gear ratio, effective radius, mass + inertia
        self.gear_eff = 0.95
        self.diff_r = 3
        self.r_e = 0.2603
        self.J_e = 10
        self.m = 350
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.Cd = 0.3
        self.A = 1.5
        self.rho = 1.2041
        self.f = 0.0130
        
        
        # State variables
        self.v = 100
        self.a = 0
        self.motor_rpm=0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 0
        self.a = 0
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0

    def step(self, throttle, delta, alpha):
        # ==================================
        #  Implement vehicle model here
        # ==================================
        T_e = throttle*np.interp(self.motor_rpm,self.motor_lookup_rpm,self.motor_lookup_tork)

        F_aero = (self.Cd * self.A * self.rho * self.v**2)/2
        R_x = self.f * self.m * self.g * np.cos(np.arctan(alpha))
        F_g = self.m * self.g * np.sin(alpha)
        F_load = F_aero + R_x + F_g
        #torque equation (angular acceleration)
        self.a=(((T_e * self.diff_r * self.gear_eff) / self.r_e) - F_load)/self.m

        #since v = a*t
        # self.v += self.a * self.sample_time
        self.v = 5
        #since x = v*t - (1/2)*a*t^2
        self.xc += self.v * np.cos(self.theta) * self.sample_time
        self.yc += self.v * np.sin(self.theta) * self.sample_time
        self.theta += (self.v / self.L) * np.tan(delta) * self.sample_time
        
        #tire velocity to RPM
        self.tire_rpm = (self.v * 60)/(2*np.pi*self.r_e)
        self.motor_rpm = max(600, min(self.tire_rpm * self.diff_r, 6000))


sample_time = 0.01
time_end = 50
model = Vehicle()



print(refpose123)

kontrol=TrajectoryProcess(refpose123)

t_data = np.arange(0,time_end,sample_time)
v_data = np.zeros_like(t_data)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

# throttle percentage between 0 and 1
throttle = 0.2

# incline angle (in radians)
alpha = 0
for i in range(t_data.shape[0]):
    v_data[i] = model.v
    x_data[i] = model.xc
    y_data[i] = model.yc
    print("v:",model.v)
    delta = kontrol.process_poses_stanley([model.xc,model.yc],model.theta,model.v)
    model.step(throttle,delta, alpha)

    
plt.plot(x_data,y_data)
plt.plot(refpose123)
plt.show()