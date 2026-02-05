import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.integrate import cumulative_trapezoid

class IMUprocess:
    
    def __init__(self, qW, qX, qY, qZ, aX, aY, aZ, delay=0.01):
        # np.column_stack trasforma le tue colonne da (N,) a una matrice (N, 3)
        # Questo permette a NumPy di processare tutti i campioni insieme velocemente
        self.__acc = np.column_stack((aX, aY, aZ))
        
        # SciPy richiede i quaternioni nel formato [x, y, z, w]
        # Li impiliamo in una matrice (N, 4)
        self.__quat = np.column_stack((qX, qY, qZ, qW))
        self.__delay = delay

    def absAcc(self):
        # 1. Creiamo l'oggetto rotazione per tutti gli N campioni
        # Se i quaternioni dal DMP non sono perfettamente unitari, 
        # aggiungiamo una normalizzazione interna.
        r = R.from_quat(self.__quat)
        
        # 2. Definiamo il vettore gravità standard (m/s^2)
        gravity_world = np.array([0, 0, 9.80665])
        
        # 3. Ruotiamo la gravità dal sistema "Mondo" al sistema "Sensore" (Body)
        # Usiamo r.inv() perché i dati dell'accelerometro sono nel sistema locale.
        # gravity_body sarà una matrice (N, 3)
        gravity_body = r.inv().apply(gravity_world)
        
        # 4. Sottraiamo la gravità orientata dall'accelerazione misurata
        # Ora la sottrazione avviene riga per riga correttamente (N, 3) - (N, 3)
        acc_linear = self.__acc - gravity_body
        
        # 5. Calcoliamo la norma (modulo) del vettore risultante per ogni istante
        # axis=1 indica che vogliamo la norma lungo le coordinate (x,y,z) per ogni riga
        abs_acc = np.linalg.norm(acc_linear, axis=1)
        
        return abs_acc 
    
    def absVelocity(self):
        # 1. Otteniamo l'accelerazione lineare (vettoriale Nx3)
        r = R.from_quat(self.__quat)
        gravity_world = np.array([0, 0, 9.80665])
        gravity_body = r.inv().apply(gravity_world)
        acc_linear = self.__acc - gravity_body
        
        # 2. Integrazione per ogni asse (X, Y, Z)
        # Usiamo l'integrazione trapezoidale per maggiore precisione
        vel_x = cumulative_trapezoid(acc_linear[:, 0], dx=self.__delay, initial=0)
        vel_y = cumulative_trapezoid(acc_linear[:, 1], dx=self.__delay, initial=0)
        vel_z = cumulative_trapezoid(acc_linear[:, 2], dx=self.__delay, initial=0)
        
        # 3. Calcolo del modulo della velocità (Velocità Assoluta)
        vel_vector = np.column_stack((vel_x, vel_y, vel_z))
        abs_vel = np.linalg.norm(vel_vector, axis=1)
        
        return abs_vel