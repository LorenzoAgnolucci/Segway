# Parameters
from math import pi

# Parameters
from control.matlab import ss, c2d, ssdata, lqr, lqe
import numpy as np
from numpy import eye, zeros

g = 9.81                            # gravity
m = 0.023  # 0.023                  # wheel mass 0.038
M = 0.687  # 0.687                  # robot mass
R = 0.027  # 0.027                  # wheel radius 0.034
Jw = m * R ** 2 / 2                 # wheel inertia
W = 0.15  # 0.15                    # width
D = 0.05  # 0.05                    # depth
H = 0.16  # 0.16                    # height
L = H / 2                           # center of mass height
J_psi = (M * L ** 2) / 3            # pitch inertial moment
J_phi = M * (W ** 2 + D ** 2) / 12  # yaw inertial moment
Jm = 1e-05                          # engine inertial moment

Rm = 6.69                           # engine resistance
Kb = 0.468                          # engine electrical constant
Kt = 0.317                          # engine torque constant
fm = 0.0022                         # engine friction
fw = 0                              # wheel-ground friction
Ts = 0.01                           # sampling time
ts1 = 0.004                         # ts1 sample time [sec]
a_d = 0.8                           # suppress velocity noise
WN_powerGs = 1.5e-6                 # gyroscope power noise
WN_Gs = 0.03                        # gyroscope band noise

n = 1  # Gear ratio

# State Space Matrices

m1 = (2 * m + M) * R ** 2 + 2 * Jw + 2 * n ** 2 * Jm
delta = M * L * R
m2 = (delta - 2 * n ** 2 * Jm)
n1 = 2 * (n * Kb * Kt / Rm + fm + fw)
n2 = 2 * (n * Kb * Kt / Rm + fm)
gamma = M * L ** 2
m4 = gamma + J_psi + 2 * n ** 2 * Jm
sigma = M * g * L
d = m1 * m4 - m2 ** 2
alfa = n * Kt / Rm
degrees = -8.7  # 8.756
cond_i = degrees * pi / 180

A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, -m2 * sigma / d, - (m4 * n1 + m2 * n2) / d, (n2 * m4 + m2 * n2) / d],
              [0, m1 * sigma / d, (m2 * n1 + n2 * m1) / d, - (n2 * m2 + n2 * m1) / d]])

B = np.array([[0, 0], [0, 0],
              [alfa * (m4 + m2) / d, alfa * (m4 + m2) / d],
              [-alfa * (m2 + m1) / d, - alfa * (m2 + m1) / d]])
Dw = np.array([[0, 0],
               [0, 0],
               [-m4 / d, m2 / d],
               [m2 / d, - m1 / d]])

Dw1 = np.array([[0, 0],
                [0, 0],
                [1, 0],
                [0, 1]])

C = np.array([[1, 0, 0, 0],
              [0, 0, 0, 1]])

DD = np.array([[0, 0],
               [0, 0]])

# process and measurement noise

w1 = 1  # voltage left noise power
w2 = 1  # voltage right noise power

VarGyro = WN_powerGs / WN_Gs    # gyroscope noise power variance
VarEnc = 1e-7                   # steer encoder measure noise variance
var_w1 = w1 / Ts                # steer torque process noise variance
var_w2 = w2 / Ts                # lean torque process noise variance

Qn = np.array([[var_w1, 0],
               [0, var_w2]])    # process noise covariance
Rn = np.array([[VarEnc, 0],
               [0, VarGyro]])   # measure noise covariance

# LQR control

A_bar = np.vstack((np.concatenate((A, zeros((4, 1))), axis=1),
                       np.array([1, 0, 0, 0, 0])))
B_bar = np.vstack((B, [0, 0]))
Q1 = np.array([[1, 0, 0, 0, 0],
               [0, 1, 0, 0, 0],
               [0, 0, 1, 0, 0],
               [0, 0, 0, 1, 0],
               [0, 0, 0, 0, 50]])
P1 = 100. * eye(2)
sys = ss(A, B, C, DD)
sysd = c2d(sys, Ts, 'zoh')
Ad, Bd, Cd, Dd = ssdata(sysd)
Kd, Sd, Ed = lqr(A_bar, B_bar, Q1, P1)
Kx = Kd[0, 0:4]  # Kd(1, 1:4)
Ki = Kd[0, 4]
Kx[0, 2] = Kx[0, 2] * 0.8

# Kalman Filter

C1 = np.array([[1, 0, 0, 0],
               [0, 0, 0, 1]])
sys2 = ss(A, np.hstack((B, Dw)), C1, zeros((2, 4)))
sysd2 = c2d(sys2, Ts, 'zoh')
# KESTD, Lk, Pk = lqe(sys2.A, None, sys2.C, Qn, Rn)  # matlab version used 'delayed' estimator, here we use default provided by `python-control`
# KESTD = c2d(KESTD, Ts, 'zoh')
# Ak, Bk, Ck, Dk = ssdata(KESTD)
