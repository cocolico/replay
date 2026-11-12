"""Aubo modified-DH model, using read-back calibrated controller parameters."""
import numpy as np
from scipy.spatial.transform import Rotation


def placements(q,parameters):
    q=np.asarray(q,dtype=float)
    if q.shape!=(6,) or not np.all(np.isfinite(q)):raise ValueError('Six finite radians required')
    out=[];T=np.eye(4)
    for i in range(6):
        alpha,a,d,theta,beta=[float(parameters[k][i]) for k in ('alpha','a','d','theta','beta')]
        X=np.eye(4);X[:3,:3]=Rotation.from_euler('x',alpha).as_matrix();X[0,3]=a
        Z=np.eye(4);Z[:3,:3]=Rotation.from_euler('z',theta+q[i]).as_matrix();Z[2,3]=d
        B=np.eye(4);B[:3,:3]=Rotation.from_euler('y',beta).as_matrix()
        T=T@X@Z@B;out.append(T.copy())
    return out


def fk(q,parameters):return placements(q,parameters)[-1]
