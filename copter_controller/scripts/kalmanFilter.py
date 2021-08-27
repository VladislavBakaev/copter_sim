import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter():  
    def __init__(self, n):
        self.Q = np.array([0.0001]*n)
        self.R = np.array([0.1]*n)
        self.P_k_k1 = np.array([1]*n)
        self.Kg = np.array([0]*n)
        self.P_k1_k1 = np.array([1]*n)
        self.x_k_k1 = np.array([0]*n)
        self.ADC_OLD_Value = np.array([0]*n)
        self.kalman_adc_old = np.array([0]*n)
        self.weight_k = 0.5
    
    def getValue(self, ADC_value):
        ADC_value = np.array(ADC_value)
        Z_k = ADC_value

        if (abs(max(self.kalman_adc_old-ADC_value))>=30):
            x_k1_k1= ADC_value*(1-self.weight_k) + self.kalman_adc_old*self.weight_k
        else:
            x_k1_k1 = self.kalman_adc_old

        self.x_k_k1 = x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.Q

        self.Kg = self.P_k_k1/(self.P_k_k1 + self.R)

        kalman_adc = self.x_k_k1 + self.Kg * (Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg)*self.P_k_k1
        self.P_k_k1 = self.P_k1_k1

        self.ADC_OLD_Value = ADC_value
        self.kalman_adc_old = kalman_adc
        return kalman_adc.tolist()

# array1 = np.array([100]*200)

# s1 = np.random.normal(0, 5, 200)

# test_array1 = array1 + s1
# plt.plot(test_array1)
# adc1=[]

# array2 = np.array([200]*200)

# s2 = np.random.normal(0, 5, 200)

# test_array2 = array2 + s2
# plt.plot(test_array2)
# adc2=[]

# kalman = KalmanFilter(2)

# for i in range(200):
#     filtred = kalman.getValue(np.array([test_array1[i], test_array2[i]]))
#     adc1.append(filtred[0])
#     adc2.append(filtred[1])

# plt.plot(adc1)   
# plt.plot(array1)

# plt.plot(adc2)   
# plt.plot(array2)   
# plt.show()