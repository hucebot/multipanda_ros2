import numpy as np

# =========================
# GEOMETRY
# =========================

h_fts = 0.035 # [m]
R_fts = 0.035 # [m]

# =========================
# MASSES
# =========================

m_gripper = 0.728 # [Kg]
m_fts = 0.236 # [Kg]
m_realsense = 0.098 # [Kg]

total_mass = m_gripper + m_fts + m_realsense

# =========================
# COM positions wrt link8
# =========================

# gripper (from GUI/datasheet)
p_gripper_wo_fts = np.array([-0.01, 0.0, 0.03]) # [m]

# gripper shifted because now FTS inserted
p_gripper = p_gripper_wo_fts + np.array([0.0, 0.0, h_fts]) # [m]

# FTS (cilinder approximation)
p_fts = np.array([0.0, 0.0, h_fts/2]) # [m]

# =========================
# NEW COM wrt link 8 (flange)
# =========================

new_p = (m_fts*p_fts + m_gripper*p_gripper) / (m_fts + m_gripper) # [m]

# =========================
# FTS inertia wrt its COM
# =========================

Ixx = (1/12) * m_fts * (3*R_fts**2 + h_fts**2)
Iyy = Ixx
Izz = 0.5 * m_fts * R_fts**2

I_fts = np.diag([Ixx, Iyy, Izz])

# =========================
# Gripper inertia wrt its COM
# =========================

I_gripper = np.array([
    [0.001, 0.0, 0.0],
    [0.0, 0.0025, 0.0],
    [0.0, 0.0, 0.0017]
])  

# =========================
# Parallel axis function
# =========================

def shift_inertia(I_com, m, d):

    d2 = np.dot(d,d)
    return I_com + m*(d2*np.eye(3) - np.outer(d,d))

# =========================
# Shift to new COM
# =========================

d_fts = p_fts - new_p
d_gripper = p_gripper - new_p

I_fts_new = shift_inertia(I_fts, m_fts, d_fts)
I_gripper_new = shift_inertia(I_gripper, m_gripper, d_gripper)

# =========================
# TOTAL inertia
# =========================

I_total = I_fts_new + I_gripper_new

print("Total mass: ", total_mass)
print("New COM:", new_p)
print("Total inertia wrt new COM:")
print(I_total)