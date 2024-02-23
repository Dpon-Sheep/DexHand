import numpy as np

class DexHand_kEstimator():
    PRELOAD_CNT = 20
    def __init__(self):
        self._data_cnt = 0 
        self._P = np.zeros((2,2))
        self._Phi = np.ones((DexHand_kEstimator.PRELOAD_CNT,2))
        self._y = np.ones((DexHand_kEstimator.PRELOAD_CNT))
        self._K = np.zeros((2,2))
        self.k = 0.07
        self._pos0 = -1
        self._minff = 1

    def data_push_in(self,nowpos,nowforce):
        if self._data_cnt == 0:
            self._pos0 = nowpos-20
        nowpos = nowpos-self._pos0

        if self._data_cnt < DexHand_kEstimator.PRELOAD_CNT:
            self._Phi[self._data_cnt][0] = nowpos
            self._y[self._data_cnt] = nowforce
            self._data_cnt += 1
            if self._data_cnt == DexHand_kEstimator.PRELOAD_CNT:
                self.theta = np.polyfit(self._Phi[:,0],self._y,deg=1)
                self._P = np.linalg.inv(self._Phi.T @ self._Phi)
                self._last_pos = nowpos
            return
        
        phi = np.array((nowpos,1))
        err = nowforce - np.dot(self.theta,phi)
        x = 130*(abs(err)-0.08)
        lambda1 = abs(nowpos-self._last_pos)
        lambda2 = x/(1+abs(x))+1
        forgot_factor = 1-lambda1*lambda2*1.575
        if forgot_factor < 0.3:
            forgot_factor = 0.3
        if forgot_factor < self._minff:
            self._minff = forgot_factor

        self._K = np.dot(self._P,phi)/(forgot_factor+np.dot(phi,np.dot(self._P,phi)))
        self._P = ((np.eye(2)-np.outer(self._K,phi)) @ self._P ) / forgot_factor
        self.theta = self.theta +self._K * err
        self.k = self.theta[0]
        if self.k<0.05:
            self.k = 0.05

        self._last_pos = nowpos