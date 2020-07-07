# -*- coding: utf-8 -*-
"""
Created on Mon Jun 11 18:44:12 2018

@author: Francois
"""
import csv as csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.interpolate as interpolate
import matplotlib.patches as patches

#--------------------------------------------------------------------------
#                       Plots parameters
#--------------------------------------------------------------------------
from matplotlib import rcParams
from matplotlib import rc
rcParams['axes.titlepad'] = 20 
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=False)
plt.rc('font', family='serif')

#--------------------------------------------------------------------------
#                       Data importation
#--------------------------------------------------------------------------

path="C:/Users/Francois/Desktop/StarFish1/"
file="track.csv"
#Test6_0_0_8mm_movTest2_0_2mm_away
Data=[]
reader = csv.reader(open(path+file,newline=''))
for row in reader:
    Data.append(row)
n=len(Data)

Time=np.array([float(Data[i][0])-float(Data[1][0]) for i in range(1,n)])             # Time stored is in milliseconds
Xobjet=np.array([float(Data[i][1]) for i in range(1,n)])             # Xpos in motor full-steps
Yobjet=np.array([float(Data[i][2]) for i in range(1,n)])             # Ypos in motor full-steps
Zobjet=np.array([float(Data[i][3]) for i in range(1,n)])             # Zpos is in encoder units
ThetaWheel=np.array([float(Data[i][4]) for i in range(1,n)])
ZobjWheel=np.array([float(Data[i][5]) for i in range(1,n)])
ManualTracking=np.array([int(Data[i][6]) for i in range(1,n)])   # 0 for auto, 1 for manual
ImageName=np.array([Data[i][7] for i in range(1,n)])
focusMeasure=np.array([float(Data[i][8]) for i in range(1,n)])
focusPhase=np.array([float(Data[i][9]) for i in range(1,n)])
MaxfocusMeasure=np.array([float(Data[i][10]) for i in range(1,n)])
#colorR=np.array([int(Data[i][11]) for i in range(1,n)])
#colorG=np.array([int(Data[i][12]) for i in range(1,n)])
#colorB=np.array([int(Data[i][13]) for i in range(1,n)])

#--------------------------------------------------------------------------
#                              Duration
#--------------------------------------------------------------------------

duration=' ( '+str(int(round(Time[-1]/60)))+' mn '+str(int(round(Time[-1]%60)))+' s )'
#--------------------------------------------------------------------------
#                       Boundaries equilibration
#--------------------------------------------------------------------------
xmin=Xobjet.min()
xmax=Xobjet.max()

ymin=Yobjet.min()
ymax=Yobjet.max()

print(ymin,ymax)

zmin=ZobjWheel.min()
zmax=ZobjWheel.max()

if xmax-xmin>15 and (xmin<-7.5 or xmax>7.5):
    delta_x=np.mean(Xobjet)
    Xobjet=Xobjet-delta_x
    xmin-=delta_x
    xmax-=delta_x

if ymax-ymin>3 and (ymin<0 or ymax>0):
    delta_y=(np.mean(Yobjet)-1.5)
    Yobjet=Yobjet-delta_y
    ymin-=delta_y
    ymax-=delta_y
    


#--------------------------------------------------------------------------
#                            Plot the box
#--------------------------------------------------------------------------

 
def x_y_edge(ax,x_range, y_range, z_range,alpha1,alpha2):
    xx, yy = np.meshgrid(x_range, y_range)

    for value in [0, 1]:
        ax.plot_wireframe(xx, yy, z_range[value], color="b",alpha=alpha1,linewidth=0)
        ax.plot_surface(xx, yy, z_range[value], color="b", alpha=alpha2)


def y_z_edge(ax,x_range, y_range, z_range,alpha1,alpha2):
    yy, zz = np.meshgrid(y_range, z_range)

    for value in [0, 1]:
        ax.plot_wireframe(x_range[value], yy, zz, color="b",alpha=alpha1,linewidth=0.7)
        ax.plot_surface(x_range[value], yy, zz, color="b", alpha=alpha2)


def x_z_edge(ax,x_range, y_range, z_range,alpha1,alpha2):
    xx, zz = np.meshgrid(x_range, z_range)

    for value in [0, 1]:
        ax.plot_wireframe(xx, y_range[value], zz, color="b",alpha=alpha1,linewidth=0.7)
        ax.plot_surface(xx, y_range[value], zz, color="b", alpha=alpha2)


def rect_prism(ax,x_range, y_range, z_range,alpha1,alpha2):
    x_z_edge(ax,x_range, y_range, z_range,alpha1,alpha2)
    #x_y_edge(ax,x_range, y_range, z_range,alpha1,alpha2)
    y_z_edge(ax,x_range, y_range, z_range,alpha1,alpha2)
  
def draw_channel(ax,zmin,zmax):
    rect_prism(ax,np.array([-7.5, 7.5]),np.array([-7.5, 7.5]),np.array([zmin, zmax]),0,0)
    rect_prism(ax,np.array([-7.5, 7.5]),np.array([0, 3]),np.array([zmin, zmax]),1,0.1)

#--------------------------------------------------------------------------
#                       3D Trajectory
#--------------------------------------------------------------------------


aspect=(zmax-zmin)/15.

fig = plt.figure(figsize=plt.figaspect(aspect))
ax = fig.gca(projection='3d')

draw_channel(ax,zmin,zmax)

sc1=ax.scatter(Xobjet, Yobjet, ZobjWheel, marker='o',s=0.6,c=Time, cmap=plt.hot())
ax.plot(Xobjet, Yobjet, ZobjWheel,linewidth=0.4)
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
cbar1=fig.colorbar(sc1)
cbar1.set_label('Time (s)')




#--------------------------------------------------------------------------
#                       XY Trajectory
#--------------------------------------------------------------------------

fig2=plt.figure()
ax2 = fig2.add_axes([0.2,0.2,0.8,0.6])
sc2=plt.scatter(Xobjet,Yobjet,c=Time, marker='o',s=0.6)
plt.plot(Xobjet,Yobjet,linewidth=0.4)

p2 = patches.Rectangle((-7.5, 0), 15, 3,fill=False)
ax2.add_patch(p2)

plt.xlim(-8,8)
plt.ylim(-1,4)
plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.title('XY Trajectory'+duration)
cbar2=fig2.colorbar(sc2)
cbar2.set_label('Time (s)')
plt.gca().set_aspect('equal', adjustable='box')
plt.savefig(path+'XY_Trajectory.png',dpi=300)


#--------------------------------------------------------------------------
#                       XZ and YZ Trajectory
#--------------------------------------------------------------------------


fig3=plt.figure()
ax3 = fig3.add_axes([0.2,0.2,0.8,0.6])
sc3=plt.scatter(Xobjet,ZobjWheel,c=Time, marker='o',s=0.6)
plt.plot(Xobjet,ZobjWheel,color='gray',linewidth=0.4)

p3 = patches.Rectangle((-7.5, zmin-3), 15, zmax-zmin+6,fill=False)
ax3.add_patch(p3)

plt.xlabel('X (mm)')
plt.ylabel('Z (mm)')

plt.xlim(-8,8)
plt.ylim(zmin-1,zmax+1)

plt.title('XZ Trajectory'+duration)
cbar3=fig3.colorbar(sc3)
cbar3.set_label('Time (s)')
plt.gca().set_aspect('equal', adjustable='box')
plt.savefig(path+'XZ_Trajectory.png',dpi=300)


fig4=plt.figure()
ax4 = fig4.add_axes([0.2,0.2,0.8,0.6])
sc4=plt.scatter(Yobjet,ZobjWheel,c=Time, marker='o',s=0.6)
plt.plot(Yobjet,ZobjWheel,color='gray',linewidth=0.4)

p4 = patches.Rectangle((0, zmin-3), 3, zmax-zmin+6,fill=False)
ax4.add_patch(p4)

plt.xlabel('Y (mm)')
plt.ylabel('Z (mm)')

plt.xlim(-1,4)
plt.ylim(zmin-1,zmax+1)

plt.title('YZ Trajectory'+duration)
cbar4=fig4.colorbar(sc4)
cbar4.set_label('Time (s)')
plt.gca().set_aspect('equal', adjustable='box')
plt.savefig(path+'YZ_Trajectory.png',dpi=300)

#--------------------------------------------------------------------------
#                           Trajectory function of time
#--------------------------------------------------------------------------
xrange=int(np.ceil(xmax)-np.floor(xmin)+2)
yrange=int(np.ceil(ymax)-np.floor(ymin)+2)
zrange=int(np.ceil(zmax)-np.floor(zmin)+2)

plt.figure(figsize=plt.figaspect(1)*2)
grid = plt.GridSpec(xrange+yrange+zrange, 1, wspace=1, hspace=8)

ax7=plt.subplot(grid[xrange+yrange:])
plt.plot(Time,ZobjWheel)
plt.xlabel('T (s)')
plt.ylabel('Z (mm)')
plt.xlim( Time[0],Time[-1])
plt.ylim( np.floor(zmin)-1,np.ceil(zmax)+1)

ax5=plt.subplot(grid[:xrange],sharex=ax7)
plt.plot(Time,Xobjet)
plt.ylabel('X (mm)')
p5 = patches.Rectangle((Time[0]-3, -7.5), Time[-1]-Time[0]+6, 15,fill=False,linestyle='--')
ax5.add_patch(p5)
plt.xlim( Time[0],Time[-1])
plt.ylim( np.floor(xmin)-1,np.ceil(xmax)+1)
plt.setp(ax5.get_xticklabels(), visible=False)

plt.title('Trajectory projection'+duration)

ax6=plt.subplot(grid[xrange:xrange+yrange],sharex=ax7)
plt.plot(Time,Yobjet)
p6 = patches.Rectangle((Time[0]-3, 0), Time[-1]-Time[0]+6, 3,fill=False,linestyle='--')
ax6.add_patch(p6)
plt.xlim( Time[0],Time[-1])
plt.ylabel('Y (mm)')
plt.setp(ax6.get_xticklabels(), visible=False)

plt.ylim( np.floor(ymin)-1,np.ceil(ymax)+1)




plt.savefig(path+'Trajectory_in_Time.png',dpi=300)

plt.show()
"""
##--------------------------------------------------------------------------
##                         data equalization
##--------------------------------------------------------------------------

T=np.linspace(Time[0],Time[-1],len(Time))

func_X=interpolate.interp1d(Time,Xobjet)
func_Y=interpolate.interp1d(Time,Yobjet)
func_Z=interpolate.interp1d(Time,ZobjWheel)

X=func_X(T)
Y=func_Y(T)
Z=func_Z(T)


##--------------------------------------------------------------------------
##                       Diffusivity
##--------------------------------------------------------------------------
N=int(len(X)/2)
def mean_square_displacement(data,n):
    return np.mean((data[n:]-data[:-n])**2)

def mean_square_vector(n):
    return np.mean((X[n:]-X[:-n])**2+(Y[n:]-Y[:-n])**2+(Z[n:]-Z[:-n])**2)

MSDisp_X=[]
MSDisp_Y=[]
MSDisp_Z=[]
MSDisp_Vector=[]
for i in range(1,N+1):
    MSDisp_X.append(mean_square_displacement(X,i))
    MSDisp_Y.append(mean_square_displacement(Y,i))
    MSDisp_Z.append(mean_square_displacement(Z,i))
    MSDisp_Vector.append(mean_square_vector(i))



DeltaT=Time[:N]

plt.figure()
plt.plot(DeltaT,MSDisp_X, label='X')
plt.plot(DeltaT,MSDisp_Y, label='Y')
plt.plot(DeltaT,MSDisp_Z, label='Z')
plt.plot(DeltaT,MSDisp_Vector, label='Vector')
plt.title('Mean Square Displacement' )
plt.xlabel('DeltaT (s)')
plt.legend()
plt.ylabel('Mean Square Displacement')

##--------------------------------------------------------------------------
##                      Velocity autocorrelation
##--------------------------------------------------------------------------

Vx=[]
Vy=[]
Vz=[]

for i in range(len(X)-1):
    Vx.append((X[i+1]-X[i])/(T[i+1]-T[i]))
    Vy.append((Y[i+1]-Y[i])/(T[i+1]-T[i]))
    Vz.append((Z[i+1]-Z[i])/(T[i+1]-T[i])) 
Vx=np.array(Vx)
Vy=np.array(Vy)
Vz=np.array(Vz)

def autocorrelation(data,n):
    return np.sum(Vx[:-n]*Vx[n:]*(T[1]-T[0]))

autocorrelation_Vx=[]
autocorrelation_Vy=[]
autocorrelation_Vz=[]

N2=int(len(Vx)/2)
for i in range(1,N2+1):
    autocorrelation_Vx.append(autocorrelation(Vx,i))
    autocorrelation_Vy.append(autocorrelation(Vy,i))
    autocorrelation_Vz.append(autocorrelation(Vz,i))

DeltaT2=T[:N2] 

plt.figure()
plt.plot(DeltaT2,autocorrelation_Vx, label='Vx')
plt.plot(DeltaT2,autocorrelation_Vy, label='Vy')
plt.plot(DeltaT2,autocorrelation_Vz, label='Vz')
plt.title('Autocorrelation in speed function of DeltaT' )
plt.xlabel('DeltaT (s)')
plt.legend()
plt.ylabel('Speed autocorrelation function of time interval')   

##--------------------------------------------------------------------------
##                  Probability distribution of speed
##--------------------------------------------------------------------------


plt.figure()
weights_x = np.ones_like(Vx)/float(len(Vx))
weights_y = np.ones_like(Vy)/float(len(Vy))
weights_z = np.ones_like(Vz)/float(len(Vz))
plt.hist([Vx,Vy,Vz], bins=200,weights=[weights_x,weights_y,weights_z], label=['Vx','Vy','Vz']) #density=True)
plt.xlabel('speed (mm.s-1)')
plt.ylabel('probability distribution of speed')
plt.legend()
plt.title('Speed distribution')
plt.axis([-2, 2, 0, 1])
"""
##--------------------------------------------------------------------------
##                       Fourier Transform
##--------------------------------------------------------------------------


"""


def Fourier(y,t):
    n = len(t) # length of the signal
    Fs=n/(t[-1]-t[0])
    print(Fs)
    
    k = np.arange(n)
    T = n/Fs
    frq = k/T # two sides frequency range
    frq = frq[range(int(n/2))] # one side frequency range
    
    Y = np.fft.fft(y)/n # fft computing and normalization
    Y = Y[range(int(n/2))]
    
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(t,y)
    ax[0].set_xlabel('Time')
    ax[0].set_ylabel('Amplitude')
    ax[1].plot(frq,abs(Y),'r') # plotting the spectrum
    ax[1].set_xlabel('Freq (Hz)')
    ax[1].set_ylabel('|A(freq)|')
    #plt.axis([0, Fs,0,100])

Fourier(X,T)
Fourier(Y,T)
Fourier(Z,T)

#Fs1 = 150  # sampling rate
#Ts1 = 1/Fs1 # sampling interval
#t = np.arange(0,1,Ts1) # time vector
#ff = 5;   # frequency of the signal
#y = np.sin(2*np.pi*ff*t)
#Fourier(y,t)

"""


