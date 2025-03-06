import numpy as np

class Legfl(object):

    L0 = 0.0128
    L1 = 0.0118; L2 = 0.042
    L3 = 0.0128; L4 = 0.033
    L6 = 0.006; alpha1 = 20

    def calculations(L4, L6, alpha1):
        l5 = np.sqrt(L4**2 + L6**2 - 2 * np.cos(np.radians(alpha1)) * L4 * L6)
        a_2 = np.degrees(np.arccos((l5**2 + L4**2 - L6**2) / (2 * l5 * L4)))
        return l5, a_2

    L5, alpha2 = calculations(L4, L6, alpha1)

    L7 = 0.030; L8 = 0.030
    L9 = 0.006
    L10 = 0.020 # foot
    gamma = 30

class Legfr(object):

    L0 = 0.0128
    L1 = 0.0118; L2 = 0.042
    L3 = 0.0128; L4 = 0.033
    L6 = 0.006; alpha1 = 20

    def calculations(L4, L6, alpha1):
        l5 = np.sqrt(L4**2 + L6**2 - 2 * np.cos(np.radians(alpha1)) * L4 * L6)
        a_2 = np.degrees(np.arccos((l5**2 + L4**2 - L6**2) / (2 * l5 * L4)))
        return l5, a_2

    L5, alpha2 = calculations(L4, L6, alpha1)

    L7 = 0.030; L8 = 0.030
    L9 = 0.006
    L10 = 0.020 # foot
    gamma = 30

class Leghl(object):

    L0 = 0.0128
    L1 = 0.0118; L2 = 0.042
    L3 = 0.0128; L4 = 0.033
    L6 = 0.006; alpha1 = 20

    def calculations(L4, L6, alpha1):
        l5 = np.sqrt(L4**2 + L6**2 - 2 * np.cos(np.radians(alpha1)) * L4 * L6)
        a_2 = np.degrees(np.arccos((l5**2 + L4**2 - L6**2) / (2 * l5 * L4)))
        return l5, a_2

    L5, alpha2 = calculations(L4, L6, alpha1)

    L7 = 0.030; L8 = 0.030
    L9 = 0.006
    L10 = 0.020 # foot
    gamma = 30

class Leghr(object):

    L0 = 0.0128
    L1 = 0.0118; L2 = 0.042
    L3 = 0.0128; L4 = 0.033
    L6 = 0.006; alpha1 = 20

    def calculations(L4, L6, alpha1):
        l5 = np.sqrt(L4**2 + L6**2 - 2 * np.cos(np.radians(alpha1)) * L4 * L6)
        a_2 = np.degrees(np.arccos((l5**2 + L4**2 - L6**2) / (2 * l5 * L4)))
        return l5, a_2

    L5, alpha2 = calculations(L4, L6, alpha1)

    L7 = 0.030; L8 = 0.030
    L9 = 0.006
    L10 = 0.020 # foot
    gamma = 30 





