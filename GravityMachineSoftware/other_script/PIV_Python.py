# -*- coding: utf-8 -*-
"""
Created on Thu May 10 23:00:36 2018

@author: Francois
"""

#PIV Detailed exemple

'''
from openpiv import tools, process, validation, filters, scaling, pyprocess
import numpy as np
import matplotlib.pyplot as plt
%matplotlib inline
from IPython.display import display
from ipywidgets import interact_manual, interactive, fixed, IntSlider, HBox, VBox, Layout

winsize = 24 # pixels
searchsize = 64  # pixels, search in image B
overlap = 12 # pixels
dt = 0.02 # sec


#Run the OpenPIV (fast code, precompiled in Cython)
u, v, sig2noise = process.extended_search_area_piv( frame_a.astype(np.int32), frame_b.astype(np.int32), 
                                                     window_size=winsize, overlap=overlap, dt=dt, 
                                                     search_area_size=searchsize, sig2noise_method='peak2peak' )

#Get a list of coordinates for the vector field
x, y = process.get_coordinates( image_size=frame_a.shape, window_size=winsize, overlap=overlap )

#Clean the peaks that are below a quality threshold
u, v, mask = validation.sig2noise_val( u, v, sig2noise, threshold = 1.3 )

#Replace those that are masked as bad vectors with local interpolation
u, v = filters.replace_outliers( u, v, method='localmean', max_iter=10, kernel_size=2)

#Scale the results from pix/dt to mm/sec
x, y, u, v = scaling.uniform(x, y, u, v, scaling_factor = 96.52 )

#store the result in a text file
tools.save(x, y, u, v, mask, 'exp1_001.txt' )

#plot the data stored in the text file
tools.display_vector_field('exp1_001.txt', scale=100, width=0.0025)
'''
#Another exemple
'''
winsize = 32 # pixels
searchsize = 64  # pixels, search in image B
overlap = 16 # pixels
dt = 1.0 # sec
u0, v0, sig2noise = process.extended_search_area_piv( frame_a.astype(np.int32), frame_b.astype(np.int32), window_size=winsize, overlap=overlap, dt=dt, search_area_size=searchsize, sig2noise_method='peak2peak' )
x, y = process.get_coordinates( image_size=frame_a.shape, window_size=winsize, overlap=overlap )
u, v, mask = validation.sig2noise_val( u0, v0, sig2noise, threshold = 1.1 )
u, v = filters.replace_outliers( u, v, method='localmean', max_iter=10, kernel_size=2)
# x, y, u, v = scaling.uniform(x, y, u, v, scaling_factor = 96.52 )

plt.figure(figsize=(10,8))
plt.quiver(x,y,u,v,color='b')
plt.quiver(x[mask],y[mask],u[mask],v[mask],color='r')
'''