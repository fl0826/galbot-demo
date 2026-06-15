import os
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
import h5py
import scipy.io as sio
from sensor_msgs.msg import Image, JointState
import message_filters

class SaveData2HDF5:
    def __init__(self,path,filename):
        os.makedirs(path, exist_ok=True)  # 如果路径不存在，先创建
        self.h5file = h5py.File(os.path.join(path, filename + ".h5"), "w")
        self.group  = {}
        self.group_index  = {}
    
    def save_data_numpy(self,variable_name,value):
        if self.group.get(variable_name,0) == 0:
            self.group[variable_name] = self.h5file.create_group(variable_name)
            self.group_index[variable_name] = 0
            
        self.group[variable_name].create_dataset(str(self.group_index[variable_name]), data=value, dtype=np.float64, compression="gzip")
        self.group_index[variable_name] = self.group_index[variable_name] + 1
        
    def stop_save(self):
        if(len(self.group)>0):
            self.h5file.close() 
            
class SaveData2Mat: #numpy和matlab对高维度矩阵的定义有点差别
    def __init__(self,path,filename):
        os.makedirs(path, exist_ok=True)  # 如果路径不存在，先创建
        self.fullpath = os.path.join(path, filename + ".mat")
        self.group  = {}
        
    def save_data_numpy(self,variable_name,value):
        if self.group.get(variable_name,0) == 0:
            self.group[variable_name] = []
            
        self.group[variable_name].append(value.astype(np.float64))
        
    def stop_save(self):
        if (len(self.group)>0):
            for key in self.group.keys():
                tmp = np.stack(self.group[key], axis=-1)#-1的时候是按matlab的形式保存的，但是python显示的会很奇怪
                self.group[key] = tmp
        
            sio.savemat(self.fullpath, self.group)
            
class ReadDataFromMat:
    def __init__(self,fullpath):
        self.data = sio.loadmat(fullpath)  
        exclude_keys = {"__header__", "__version__", "__globals__"}
        
        for key in self.data.keys() - exclude_keys:            
            if self.data[key].ndim == 1:#没有一维矩阵，至少是二维矩阵
                pass
            
            if self.data[key].ndim == 2:#二维矩阵不需要改
                pass
            
            if self.data[key].ndim == 3:
                tmp = np.transpose(self.data[key], (2, 0, 1))
                self.data[key] = tmp