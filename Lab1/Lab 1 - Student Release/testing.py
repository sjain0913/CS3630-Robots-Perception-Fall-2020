import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color, transform
import ransac_score
import matplotlib.pyplot as plt

# PREPROCESSING
image = io.imread('a.bmp', as_gray=True)
image = filters.gaussian(image, sigma=(0.6,0.6))
thresh = filters.threshold_minimum(image)
binary = image > thresh

# HOG
(H,hog_image) = feature.hog(binary, orientations=8, pixels_per_cell=(32,32), cells_per_block=(2,2), visualize=True, multichannel=False)

# print(H.size)
# io.imshow(hog_image)
# plt.show()
