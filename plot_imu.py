
import numpy as np
import matplotlib.pyplot as plt
fd = open ('gait3.dat')
data = [map(float,line.split()) for line in fd]
m_data = np.array(data, np.float16)
x = range(len(m_data[:,0]))

for i in xrange(1,len (m_data[1,:])):
    plt.plot(x, m_data[:,i],label=("Line" + str(i)))

plt.legend()
plt.show()
