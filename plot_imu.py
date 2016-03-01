#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, linspace, pi
from pylab import plot, show, title, xlabel, ylabel, subplot
from scipy import fft, arange
def plotSpectrum(y,Fs):
     """
     Plots a Single-Sided Amplitude Spectrum of y(t)
     """
     n = len(y) # length of the signal
     k = arange(n)
     T = n/Fs
     frq = k/T # two sides frequency range
     frq = frq[range(n/2)] # one side frequency range

     Y = fft(y)/n # fft computing and normalization
     Y = Y[range(n/2)]
     
     plt.plot(frq,abs(Y),'r') # plotting the spectrum
     xlabel('Freq (Hz)')
     ylabel('|Y(freq)|')

#fd = open ('up.dat')
fd = open ('filt.dat')

data = [map(float,line.split()) for line in fd]
m_data = np.array(data, np.float32)
x = range(len(m_data[:,0]))

#plotSpectrum(m_data[:,1],15)

for i in xrange(0,len (m_data[1,:])):
    plt.plot(x, m_data[:,i],label=("Line" + str(i)))

plt.legend()
plt.show()

