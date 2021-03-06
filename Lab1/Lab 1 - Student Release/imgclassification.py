#!/usr/bin/env python

##############
#### Your name: Saumya Jain
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color, transform, measure
import ransac_score

class ImageClassifier:
    
    def __init__(self):
        self.classifier = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)
        
        #create one large array of image data
        data = io.concatenate_images(ic)
        
        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]
        
        return(data,labels)

    def extract_image_features(self, data):
        # Please do not modify the header above

        # extract feature vector from image data
        
        feature_data = []

        for i in data:
            # 1. PREPROCESSING: grayscaling
            i = color.rgb2gray(i)
            # 1. PREPROCESSING: gaussian blur
            i = filters.gaussian(i, sigma=(0.6,0.6))
            # 1. PREPROCESSING: thresholding to get black and white (as displayed in pdf)
            thresh = filters.threshold_minimum(i)
            binary_image = i > thresh

            # Couldn't resolve error related to putting a boolean image array inside hog, had to use image without thresholding
            # 2. FEATURE DESCRIPTOR: HOG
            (feature_i, hog_image) = feature.hog(i, orientations=8, pixels_per_cell=(32,32), cells_per_block=(2,2), visualize=True, multichannel=False)
            
            feature_data.append(feature_i)
            # FOR TESTING PURPOSES ONLY
            # print(H.size)
            # io.imshow(hog_image)
            # plt.show()

        feature_data = np.array(feature_data)
        # Please do not modify the return type below
        return feature_data

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above
        
        clf = svm.SVC()
        clf.fit(train_data, train_labels)

        # Saving trained model to self.classifier
        self.classifier = clf

    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels

        predicted_labels = self.classifier.predict(data)

        # Please do not modify the return type below
        return predicted_labels

    def line_fitting(self, data):
        # Please do not modify the header

        # fit a line to the arena wall using RANSAC
        # return two lists containing slopes and y intercepts of the line
        slope = []
        intercept = []
        for i in data:
            # 1. EDGE DETECTION: Using Canny on grayscale images
            i = color.rgb2gray(i)
            i = feature.canny(i, sigma=3)

            # 2. RANSAC: Run on output of edge detection
            required = np.argwhere(i == True)
            required = np.flip(required, axis=1)
            model_robust = measure.ransac(required, measure.LineModelND, min_samples=2, residual_threshold=0.86)
            temp_slope = model_robust[0].params[1][1] / model_robust[0].params[1][0]
            temp_intercept = model_robust[0].params[0][1] - (temp_slope * model_robust[0].params[0][0])

            slope.append(temp_slope)
            intercept.append(temp_intercept)

        # Please do not modify the return type below
        return slope, intercept
        
def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    (wall_raw, _) = img_clf.load_data_from_folder('./wall/')
    
    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))
    
    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTest results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))

    # ransac
    print("\nRANSAC results")
    print("=============================")
    s, i = img_clf.line_fitting(wall_raw)
    print(f"Line Fitting Score: {ransac_score.score(s,i)}/10")

if __name__ == "__main__":
    main()
