import numpy as np

def wrap_to_pi(angle):
    """Wrap angle(s) to [-pi, pi]. Accepts float or np.array."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd, n_joints=6, u_max=1.0, i_max=0.5, backcalc_beta=0.2):
        self.Kp = np.array(Kp, dtype=float)
        self.Ki = np.array(Ki, dtype=float)
        self.Kd = np.array(Kd, dtype=float)
        self.n = n_joints
        self.int_err = np.zeros(n_joints)
        self.u_max = np.array([u_max]*n_joints, dtype=float) if np.isscalar(u_max) else np.array(u_max, dtype=float)
        self.i_max = np.array([i_max]*n_joints, dtype=float) if np.isscalar(i_max) else np.array(i_max, dtype=float)
        self.backcalc_beta = backcalc_beta  # anti-windup

    def reset(self):
        self.int_err[:] = 0.0

    def step(self, q_des, q, qd_meas, dt):
        # position error wrapped to [-pi, pi]
        e = wrap_to_pi(q_des - q)

        self.int_err += e * dt
        self.int_err = np.clip(self.int_err, -self.i_max, self.i_max)
        u = self.Kp * e + self.Ki * self.int_err - self.Kd * qd_meas
        u_sat = np.clip(u, -self.u_max, self.u_max)
        aw = u_sat - u
        self.int_err += self.backcalc_beta * aw
        
        return u_sat, e