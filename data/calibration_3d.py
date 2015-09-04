import numpy as np
import cv2


source = np.empty([1,3],np.float32)#eros(shape, np.float32)
target = np.empty([1,3],np.float32)

# [x, y, z]
"""
source[0][0] = [857, 120, 854]
source[0][1] = [254, 120, 855]
source[0][2] = [256, 120, 255]
source[0][3] = [858, 120, 255]
target = source * 10
"""
path = "/home/ir/perception_ws/src/vicon_with_camera/data/"
img_p = path + "points%d.txt"
vic_p = path + "vicon%d.txt"
for i in range(0,10):
    si = img_p % (i)
    sv = vic_p % (i)
#    target = []
#    source = []
    with open(si) as fi:
        data = fi.readlines()
        for d in data:
            w = d.split()
            s = np.array([],np.float32)
            for wi in w:
                s = np.insert(s,len(s),float(wi))
            s = np.insert(s,len(s),1)
            target = np.vstack((target,np.atleast_2d(s)))
    with open(sv) as gi:
        data = gi.readlines()
        for d in data:
            w = d.split()
            s = np.array([],np.float32)
            for wi in w:
                s = np.insert(s,len(s),float(wi))
            source = np.vstack((source,np.atleast_2d(s)))
target = target[1:target.shape[0],:]            
source = source[1:source.shape[0],:]            

retval, M, inliers = cv2.estimateAffine3D(source, target)

for i in range (0,source.shape[0]):
    d = source[i,:]
    d = np. insert(d,len(d),1)
#    print np.mat(M)
#    print np.mat(d)
#    print '\n'
    m = np.mat(M) * np.mat(d).T
    print m.T
    print target[i,:]
    print '\n'


#print M
