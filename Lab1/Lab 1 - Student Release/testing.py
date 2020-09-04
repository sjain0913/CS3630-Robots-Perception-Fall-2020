import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color, transform, measure
import ransac_score
import matplotlib.pyplot as plt

# PREPROCESSING
# image = io.imread('a.bmp', as_gray=True)
# image = filters.gaussian(image, sigma=(0.6,0.6))
# thresh = filters.threshold_minimum(image)
# binary = image > thresh

# HOG
# (H,hog_image) = feature.hog(binary, orientations=8, pixels_per_cell=(32,32), cells_per_block=(2,2), visualize=True, multichannel=False)

# print(H.size)
# io.imshow(hog_image)
# plt.show()

image = io.imread('wall.bmp', as_gray=True)
data = feature.canny(image, sigma=3)
nonzero = np.argwhere(data == True)
nonzero = np.flip(nonzero, axis=1)
model_robust = measure.ransac(nonzero, measure.LineModelND, min_samples=2, residual_threshold=1)
print(model_robust[0].params)
slope = model_robust[0].params[1][1] / model_robust[0].params[1][0]
intercept = model_robust[0].params[0][1] - (slope * model_robust[0].params[0][0])
# first is array with origin coordinates
# second is unit vector which gives slope

print(slope)
print(intercept)

# io.imshow(model_robust)
# plt.show()