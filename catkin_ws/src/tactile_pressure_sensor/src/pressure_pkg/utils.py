# import necessary pkgs
import numpy as np
import scipy
import math

from scipy.optimize import minimize
from scipy.signal import butter, filtfilt

from math import sqrt, pi, exp


def HerzianContactLoc(pressureArray, shape, spacing, t=6,n=9):
    """
    n: amount of points used
    """    
    amount = int(shape[0]*shape[1])
    array  = np.zeros((amount,4))  # n points with highest pressure; 1: sensor number; 2: pressure; 3: x coordinate; 4: y coordinate
    
    for i in range(amount):
        array[i][0] = np.argmax(pressureArray)
        array[i][1] = pressureArray[int(array[i][0])]
        pressureArray[int(array[i][0])] = -10**9
        
    for i in range(amount):
        idx = array[i][0]
        x = spacing/2 + spacing*(idx % shape[0])
        y = spacing*shape[1] - (spacing/2 + spacing*math.floor(idx/shape[0]))
        array[i][2] = x
        array[i][3] = y
    
    ### normalize
    length_n = max(shape[0]*spacing,shape[1]*spacing)
    array_n = np.zeros(array.shape)
    array_n.T[0] = array.T[0]
    array_n.T[1] = array.T[1].T/np.max(array.T[1])
    array_n.T[2] = array.T[2]/(length_n)
    array_n.T[3] = array.T[3]/(length_n)
    
    diff_x = False
    diff_y = False
    
    nx = 3
    while (not diff_x) and (not diff_y):
        for i in range(nx):
            for j in range(i):
                if array[i][2] != array[j][2]:
                    diff_x = True
                if array[i][3] != array[j][3]:
                    diff_y = True
        if not (diff_x and diff_y):
            nx += 1
    
    x0 = [array_n[0][1],0.1,(array_n.T[2][0:nx].max()+array_n.T[2][0:nx].min())/2,(array_n.T[3][0:nx].max()+array_n.T[3][0:nx].min())/2]
        
    bnds = scipy.optimize.Bounds([x0[0],0.0,array_n.T[2][0:nx].min(),array_n.T[3][0:nx].min()],[float('inf'),float('inf'),array_n.T[2][0:nx].max(),array_n.T[3][0:nx].max()])
    
    meth = ['Nelder-Mead','L-BFGS-B','Powell','TNC','SLSQP']
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html
    opt = {}
    opt_0 = {'maxfev':10**3}
    opt_1 = {'ftol':1e-9,'gtol':1e-7,'maxls':40}
    opt_4 = {'ftol':1e-12,'maxiter':2*10**4}
    
    E = minimize(ObjFun,x0,args=(array_n,n),method = meth[0],bounds=bnds,options=opt_0)
    
    ### denormalize
    E.x[0] = E.x[0]*np.max(array.T[1])
    E.x[2] = E.x[2]*length_n
    E.x[3] = E.x[3]*length_n

    return array, E


def ObjFun(par,array,n):
    error = 0
    for i in range(n):       
        sigma = gaussian(array[i][2],array[i][3],par[0],par[1],par[2],par[3])
        error += abs(sigma-array[i][1])
    return error


def gaussian(x,y,p0,std,x0,y0):
    return p0*exp(-0.5*(((x-x0)**2+(y-y0)**2)/(std**2+1e-6)))


def butter_lowpass_filter(data,on, freq=30):
    if on:
        fs     = freq
        cutoff = 5
        nyq    = 0.5*fs
        order  = 2
        
        normal_cutoff = cutoff/nyq
        b, a = butter(order, normal_cutoff,btype='low', analog=False)        
        y = scipy.signal.lfilter(b,a,data)
    else:
        y = data

    return y


def SlipDetectionDist(array_x,array_y,frac=0.1,dist_thres=0.1,use_last=-1):
    if use_last != -1:
        array_x = array_x[-use_last:]
        array_y = array_y[-use_last:]
    
    ind = int(len(array_x)*frac)
    
    x_mean = sum(array_x[:ind])/len(array_x[:ind])
    y_mean = sum(array_y[:ind])/len(array_y[:ind])
    
    new_x_mean = sum(array_x[ind:])/len(array_x[ind:])
    new_y_mean = sum(array_y[ind:])/len(array_y[ind:])
    
    dist = sqrt((x_mean-new_x_mean)**2+(y_mean-new_y_mean)**2)
    
    theta = math.atan2(array_y[-1]-y_mean,array_x[-1]-x_mean)/pi*180
    
    if dist > dist_thres:
        return True, dist, theta
    else:
        return False, dist, theta


def std_gaussian(x,y,p0,std,x0,y0):
    return p0/(std*sqrt(2*pi)+1e-6)*exp(-0.5*(((x-x0)**2+(y-y0)**2)/(std**2+1e-6)))


