 #!/usr/bin/env python

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
import joblib
import math

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
        # print("data shape = ", data.shape)
        l = []
        for im in data:
            im_gray = color.rgb2gray(im)

            im_gray = filters.gaussian(im_gray, sigma=1.2)

            f = feature.hog(im_gray, orientations=10, pixels_per_cell=(48, 48), cells_per_block=(2, 2), feature_vector=True, block_norm='L2-Hys')
            l.append(f)

        feature_data = np.array(l)
        # print("feature array shape = ", feature_data.shape)
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # print("shape of train data = ", train_data.shape)
        self.classifier = svm.LinearSVC()
        self.classifier.fit(train_data, train_labels)
        joblib.dump(self.classifier, 'trained_model.pkl')

    def predict_labels(self, data):
        predicted_labels = self.classifier.predict(data)
        return predicted_labels

def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./feed/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)

    
    # test model
    predicted_labels = img_clf.predict_labels(test_data)

    for i in range(len(test_labels)):
        if(test_labels[i] != predicted_labels[i]):
            print(i)


if __name__ == "__main__":
    main()
