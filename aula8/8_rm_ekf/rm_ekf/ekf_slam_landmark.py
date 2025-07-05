# based on https://github.com/C2SR/pyarena

import numpy as np
import time

from pyarena.vehicles.unicycle import Unicycle
from pyarena.world.landmark_world import LandmarkWorld
from pyarena.sensors.landmark_sensor import LandmarkSensor  
from pyarena.sensors.velocity_sensor import VelocitySensor  
from pyarena.plots.landmark_slam import LandmarkSLAM
from pyarena.slam.ekf_slam_landmark import EKFSLAMLandmark

# Setting seed for replication of experiment
np.random.seed(0)

# Vehicle 
x0 = np.array([0.0,0.0,0])
kwargsUnicycle = {'x0': x0}
mvehicle = Unicycle(**kwargsUnicycle)

# World 
nb_landmarks = 4
kwargsWorld = {'width': 5, 'height': 5, 'nb_landmarks': nb_landmarks}
mworld = LandmarkWorld(**kwargsWorld)
#mworld.landmarks['coordinate'] = np.array([[2.,1.],[4.,1.],[6,1],[8,1],
#                                           [8.,4.],[8.,6.],[2.,4.],[2.,6.],[4,8],
#                                           [2.,9.],[4.,9.],[6,9],[8,9]]).T
mworld.landmarks['coordinate'] = np.array([[1.,1.],[5.,1.],[5.,5.],[1.,5.]]).T
# Velocity Sensor
motion_noise = np.array([0.05,0.05])
kwargsVelocitySensor = {'noise': motion_noise}
mvelocity_sensor = VelocitySensor(**kwargsVelocitySensor)

# Landmark Sensor
measurement_noise = np.array([0.01,0.01])
kwargsLandmarkSensor = {'world': mworld,'max_range': 3.0, 'noise': measurement_noise}
mlandmark_sensor = LandmarkSensor(**kwargsLandmarkSensor)
# State estimation
Sigma0 = np.diag([.0,.0,.0])
kwargsEKFSLAM = {'nb_landmarks': nb_landmarks, 'Sigma0': Sigma0, 'motion_noise':motion_noise, 'measurement_noise': measurement_noise}
mestimator = EKFSLAMLandmark(**kwargsEKFSLAM)

# Plot
kwargsPlot = {'world': mworld}
mplot = LandmarkSLAM(**kwargsPlot)

# Loop
state = 0
u_vec=np.array([[0.0,0.],[4.0,0],[0.0,np.pi/2],[4.0,0.0],[0.0,np.pi/2],[4.0,0.0],[0.0,np.pi/2],[2,0]])
for u in u_vec:
    # Simulate vehicle
    #u = np.random.rand(2,1) + np.array([0.0,-.425]).reshape(2,1)
    x = mvehicle.run(dt=1,u=u)
    # Simulate sensors
    odometry = mvelocity_sensor.sample(dt=1,x=x)
    #if u[1]==0:
    if True:
        measurements = mlandmark_sensor.sample(dt=0,x=x)
    else:
        measurements = None
    # Localization
    x_est, Sigma = mestimator.run(dt=1, u=odometry, measurements=measurements)
    # Plot
    mplot.update(x_ground_truth=x,x_est=x_est,measurements=measurements, Cov=Sigma)
    time.sleep(5)

