import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.integrate import cumulative_trapezoid
from scipy import signal

class IMUprocess:
    
    def __init__(self, qW, qX, qY, qZ, aX, aY, aZ, delay_ms=10):
        self.__dt = delay_ms / 1000.0
        self.__fs = 1.0 / self.__dt
        self.__acc = np.column_stack((aX, aY, aZ))
        self.__quat = np.column_stack((qX, qY, qZ, qW))

    def accVec(self):
        r = R.from_quat(self.__quat)
        gravity_world = np.array([0, 0, 9.80665])
        gravity_body = r.inv().apply(gravity_world)
        acc_Vec = self.__acc - gravity_body
        return acc_Vec

    def absAcc(self):
        acc_Vec = self.accVec()
        return np.linalg.norm(acc_Vec, axis=1)

    def speedVec(self):
        # --- CORREZIONE QUI ---
        # NON usare absAcc() qui, altrimenti perdi le direzioni X,Y,Z.
        # Usa accVec() per mantenere la matrice (N, 3)
        acc_linear = self.accVec() 

        # --- FILTRO PASSA-ALTO ---
        sos = signal.butter(2, 0.7, 'hp', fs=self.__fs, output='sos')
        
        # sosfilt funziona su axis=-1 di default, ma qui vogliamo filtrare 
        # lungo il tempo (righe), quindi axis=0 va bene.
        acc_filtered = signal.sosfilt(sos, acc_linear, axis=0)

        # 1. Integrazione (Ora integriamo un vettore 3D, ottenendo una velocità 3D)
        vel = cumulative_trapezoid(acc_filtered, dx=self.__dt, initial=0, axis=0)

        # 2. Secondo filtraggio
        speed_vec = signal.sosfilt(sos, vel, axis=0)
        
        # Ora speed_vec è una matrice (N, 3), non un array (N,)
        return speed_vec
    
    def absSpeed(self):
        # Ora questo funzionerà perché speedVec() ritorna 3 dimensioni
        return np.linalg.norm(self.speedVec(), axis=1)