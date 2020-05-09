#!/usr/bin/env python
#coding=utf-8
###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
This module provide function to plot the speed control info from log csv file
"""
import math
import warnings

import numpy as np
import scipy.signal as signal

warnings.simplefilter('ignore', np.RankWarning)

SPEED_INTERVAL = 0.2
SPEED_DELAY = 130  #Speed report delay relative to IMU information


def preprocess(filename):
    data = np.genfromtxt(filename, delimiter=',', names=True)#把每一行数据存到列表中
    print data,data.ndim,data.shape
    #print np.where(data['io'] == 0)
    data = data[np.where(data['io'] == 0)[0]]#输出io为0的对应的索引的值，最后data是io为0的整行数据组成的列表
    #print data,data.ndim,data.shape
    data = data[np.argsort(data['time'])]#argsort返回排序后的下标
    #print data,data.ndim,data.shape
    data['time'] = data['time'] - data['time'][get_start_index(data)]#求所取数据的时间间隔，每一帧数据间隔0.01s，也就是100hz
    #print data['time'],data['time'].shape
    b, a = signal.butter(6, 0.05, 'low')#原始IMU数据通常会有噪声，添加ButterWidth来过滤某些频率的噪声，滤波器顺序和截止频率的选择可能与IMU类型相关联。
    data['imu'] = signal.filtfilt(b, a, data['imu'])
    #print data['imu'],data['imu'].shape
    #print data['imu'][-SPEED_DELAY / 10:],data['imu'][-SPEED_DELAY / 10:].shape
    #print data['imu'][0:-SPEED_DELAY / 10],data['imu'][0:-SPEED_DELAY / 10].shape
    data['imu'] = np.append(data['imu'][-SPEED_DELAY / 10:],
                            data['imu'][0:-SPEED_DELAY / 10])#把后面13个数据放到了最前面，发送给汽车的命令和反馈回来的信息之间有延迟
    #print data['imu'],data['imu'].shape
    #print data#经过预处理后的取出io为0,时间0.01s间隔，imu通过滤波器处理后的整行数据组成的列表
    return data


def get_start_index(data):
    if np.all(data['vehicle_speed'] == 0):
        return 0

    start_ind = np.where(data['brake_percentage'] == 40)

    if len(start_ind[0] > 0):
        ind = start_ind[0][0]
        while ind < len(data):
            if data['brake_percentage'][ind] == 40:
                ind += 1
            else:
                break
        return ind

    else:
        ind = 0
        while ind < len(data):
            if abs(data['vehicle_speed'][ind]) < 0.01:
                ind += 1
            else:
                break
        return ind



def process(data):
    """
    process data
    """
    np.set_printoptions(precision=3)

    if np.all(data['vehicle_speed'] == 0):
        print "All Speed = 0"
        return [], [], [], [], [], []

    start_index = get_start_index(data)

    #print "Start index: ", start_index
    data = data[start_index:]
    #print data,data.shape
    data['time'] = data['time'] - data['time'][0]
    #print data['time'],data['time'].shape
    #print data['ctlbrake'],data['ctlbrake'].shape
    #print data['ctlthrottle'],data['ctlthrottle'].shape
    #print np.diff(data['ctlbrake']) != 0,np.diff(data['ctlbrake']),np.diff(data['ctlbrake']).shape
    #print np.diff(data['ctlthrottle']) != 0,np.diff(data['ctlthrottle']),np.diff(data['ctlthrottle']).shape
    transition = np.where(
        np.logical_or(
            np.diff(data['ctlbrake']) != 0, np.diff(data['ctlthrottle']) != 0))[
                0]
    #print transition,transition.shape
    #print len(data)
    #print np.append(transition, len(data) - 1)
    transition = np.insert(np.append(transition, len(data) - 1), 0, 0)
    #print "Transition indexes: ", transition
    #print transition,transition.shape


    speedsegments = []
    timesegments = []
    accsegments = []
    tablespeed = []
    tableacc = []
    tablecmd = []
    #print len(transition)
    for i in range(len(transition) - 1):
        #print "process transition index:", data['time'][transition[i]], ":", data['time'][transition[i + 1]]
        #print data,data.shape
        speedsection = data['vehicle_speed'][transition[i]:transition[i +
                                                                      1] + 1]
        #print speedsection,speedsection.shape
        timesection = data['time'][transition[i]:transition[i + 1] + 1]
        #print data['ctlbrake']
        brake = data['ctlbrake'][transition[i] + 1]
        #print brake
        throttle = data['ctlthrottle'][transition[i] + 1]
        #print throttle
        imusection = data['imu'][transition[i]:transition[i + 1] + 1]
        #print imusection,imusection.shape
        if brake == 0 and throttle == 0:
            continue
        #print "Brake CMD: ", brake, " Throttle CMD: ", throttle
        firstindex = 0
        #print speedsection[0]
        while speedsection[firstindex] == 0:
            firstindex = firstindex + 1
        firstindex = max(firstindex - 2, 0)
        speedsection = speedsection[firstindex:]
        #print speedsection,speedsection.shape,speedsection[-1]
        timesection = timesection[firstindex:]
        #print timesection,timesection.shape
        imusection = imusection[firstindex:]
        #print imusection,imusection.shape
        if speedsection[0] < speedsection[-1]:
            is_increase = True
            lastindex = np.argmax(speedsection)
        else:
            is_increase = False
            lastindex = np.argmin(speedsection)

        speedsection = speedsection[0:lastindex + 1]
        #print speedsection,speedsection.shape
        timesection = timesection[0:lastindex + 1]
        imusection = imusection[0:lastindex + 1]

        speedmin = np.min(speedsection)
        speedmax = np.max(speedsection)
        speedrange = np.arange(
            max(0, round(speedmin / SPEED_INTERVAL) * SPEED_INTERVAL),
            min(speedmax, 30.10), SPEED_INTERVAL)#以0.2为间隔进行速度上的采样   10.01
        print len(speedrange),speedrange
        print "Speed min, max", speedmin, speedmax, is_increase, firstindex, lastindex, speedsection[-1]
        accvalue = []
        for value in speedrange:
            val_ind = 0
            if is_increase:
                while val_ind < len(
                        speedsection) - 1 and value > speedsection[val_ind]:
                    val_ind = val_ind + 1
            else:
                while val_ind < len(
                        speedsection) - 1 and value < speedsection[val_ind]:
                    val_ind = val_ind + 1
            if val_ind == 0:
                imu_value = imusection[val_ind]
            else:
                slope = (imusection[val_ind] - imusection[val_ind - 1]) / (
                    speedsection[val_ind] - speedsection[val_ind - 1])
                imu_value = imusection[val_ind - 1] + slope * (
                    value - speedsection[val_ind - 1])
            accvalue.append(imu_value)

        if brake == 0:
            cmd = throttle
        else:
            cmd = -brake
        #print "Overall CMD: ", cmd
        #print "Time: ", timesection
        #print "Speed: ", speedrange
        #print "Acc: ", accvalue
        #print cmd
        tablecmd.append(cmd)
        tablespeed.append(speedrange)
        tableacc.append(accvalue)

        speedsegments.append(speedsection)
        accsegments.append(imusection)
        timesegments.append(timesection)
        #print tablespeed
    return tablecmd, tablespeed, tableacc, speedsegments, accsegments, timesegments
