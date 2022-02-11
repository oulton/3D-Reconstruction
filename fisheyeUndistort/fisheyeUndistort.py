import cv2
import numpy as np

img = cv2.imread('1.png')
K = [[537.2958333, 0, 543.3864583],
    [0, 537.2958333, 500.8630271],
    [0, 0, 1]]
K = np.array(K)

# 畸变参数[k1, k2, p1, p2, k3=None, k4=None, k5=None, k6=None]
D = [6.8391e-002, 8.93718e-004, -2.8611e-004, -8.0824e-006]
D = np.array(D)

DIM = img.shape[:2]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)    
cv2.imwrite('unfisheyeImage.png', undistorted_img)
