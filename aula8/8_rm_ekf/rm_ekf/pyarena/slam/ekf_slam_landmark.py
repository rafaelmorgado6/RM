# based on https://github.com/C2SR/pyarena

from ..core import slam
import numpy as np

class EKFSLAMLandmark(slam.SLAM):
    def __init__(self, **kwargs):
        if 'nb_landmarks' not in kwargs:
            raise KeyError("[SLAM/LandmarkEKF] Must specify number of landmarks")

        nb_landmarks = kwargs['nb_landmarks']
        Sigma0 = kwargs['Sigma0'] if 'Sigma0' in kwargs else 1.*np.eye(3)
        motion_noise = kwargs['motion_noise'] if 'motion_noise' in kwargs else np.array([.05,.01])
        measurement_noise = kwargs['measurement_noise'] if 'measurement_noise' in kwargs else np.array([.05,.05])

        # Storing parameters
        self.nb_landmarks = nb_landmarks
        self.has_initialized_landmark = np.zeros(nb_landmarks,dtype=bool)
        # Filter variables
        self.Sigma = 100*np.eye(3+2*nb_landmarks)
        self.Sigma[0:3,0:3] = Sigma0
        self.R = np.diag(motion_noise)
        self.Q = np.diag(measurement_noise)
        self.F = np.vstack([np.eye(3), np.zeros([2*nb_landmarks,3])])        

        # Initializing parent class
        kwargsSLAM = {'x_dimension': 3+2*nb_landmarks}
        kwargs.update(kwargsSLAM)
        super().__init__(**kwargs)

    """
    Extended Kalman Filter routine
    """
    def run(self, dt, u, measurements=None):
        # Loading from previous iterations
        x_est = self.x_est
        Sigma = self.Sigma

        # Separar estado do robô
        x, y, theta = x_est[0,0], x_est[1,0], x_est[2,0]
        v, w = u[0,0], u[1,0]

        # ---------- Prediction ----------  
        # 2.
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + w * dt

        x_est[0:3, 0] = [x_new, y_new, theta_new] 

        # 3.
        G = np.eye(3 + 2*self.nb_landmarks)
        G[0,2] = -v * np.sin(theta) * dt
        G[1,2] =  v * np.cos(theta) * dt

        Rt = np.zeros((3 + 2*self.nb_landmarks, 3 + 2*self.nb_landmarks))
        Rt[0:2,0:2] = self.R * dt**2
        Rt[2,2] = self.R[1,1] * dt**2
        
        # Σ:
        Sigma = G @ Sigma @ G.T + Rt

        # Update
        # Check if there are measurements available
        # ---------- Correction ----------
        if measurements is not None and len(measurements['id']) > 0:
            for i, lm_id in enumerate(measurements['id']):
                lm_index = int(lm_id)
                dx, dy = measurements['coordinate'][0,i], measurements['coordinate'][1,i]
                lm_idx = 3 + 2*lm_index

                # Se o marco ainda não foi inicializado
                if not self.has_initialized_landmark[lm_index]:
                    theta_robot = x_est[2,0]
                    R_theta = np.array([[np.cos(theta_robot), -np.sin(theta_robot)],
                                        [np.sin(theta_robot),  np.cos(theta_robot)]])
                    landmark_global = x_est[0:2,0].reshape(2,1) + R_theta @ np.array([[dx], [dy]])
                    x_est[lm_idx:lm_idx+2, 0] = landmark_global.flatten()
                    self.has_initialized_landmark[lm_index] = True
                    continue

                # Marco já inicializado – fazer update com EKF
                lx, ly = x_est[lm_idx:lm_idx+2, 0]
                delta = np.array([[lx - x_new],
                                [ly - y_new]])
                theta_est = x_est[2,0]
                R_theta = np.array([[np.cos(theta_est), -np.sin(theta_est)],
                                    [np.sin(theta_est),  np.cos(theta_est)]])
                hi = R_theta.T @ delta  # previsão da medição

                # Jacobiano da observação
                H = np.zeros((2, 3 + 2*self.nb_landmarks))
                H[:,0:2] = -R_theta.T
                H[:,2] = R_theta.T @ np.array([[-delta[1,0]], [delta[0,0]]]).flatten()
                H[:,lm_idx:lm_idx+2] = R_theta.T

                # Ganho de Kalman
                S = H @ Sigma @ H.T + self.Q
                K = Sigma @ H.T @ np.linalg.inv(S)

                # Inovação (erro)
                zi = np.array([[dx],[dy]])
                innovation = zi - hi

                # Atualização
                x_est += K @ innovation
                Sigma = (np.eye(3 + 2*self.nb_landmarks) - K @ H) @ Sigma

        # Guardar estado atualizado
        self.x_est = np.copy(x_est)
        self.Sigma = np.copy(Sigma)

        return x_est, Sigma