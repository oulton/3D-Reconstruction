#模拟丢失部分图像数据的对齐结果
import cv2 as cv
src=cv.imread('VIS1.bmp',cv.IMREAD_GRAYSCALE)
src[250:450, 250:350] = 0

cv.imwrite('./VIS1-roi.bmp', src)