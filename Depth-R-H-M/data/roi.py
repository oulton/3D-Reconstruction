#模拟丢失部分图像数据的对齐结果
import cv2 as cv
src=cv.imread('depth1.png')
src[320:370, 240:290] = 0
cv.imwrite('./depth.png', src)