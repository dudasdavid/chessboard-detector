# import the necessary packages
from keras.models import Sequential
from keras.layers.convolutional import Conv2D, MaxPooling2D 
from keras.layers.core import Activation, Flatten, Dense
from keras import backend as K
from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing.image import img_to_array
from keras.optimizers import Adam
from sklearn.model_selection import train_test_split
from keras.utils import to_categorical
import tensorflow as tf
from imutils import paths
import numpy as np
import argparse
import random
import cv2
import os
import matplotlib
import matplotlib.pyplot as plt

image_size = 100

class LeNet:
  @staticmethod
  def build(width, height, depth, classes):
    # initialize the model
    model = Sequential()
    inputShape = (height, width, depth)
    
    # first set of CONV => RELU => POOL layers
    model.add(Conv2D(20, (5, 5), padding="same", use_bias=False,
      input_shape=inputShape))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
    
    # second set of CONV => RELU => POOL layers
    model.add(Conv2D(50, (5, 5), padding="same", use_bias=False))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
    
    # first (and only) set of FC => RELU layers
    model.add(Flatten())
    model.add(Dense(500))
    model.add(Activation("relu"))
    
    # softmax classifier
    model.add(Dense(classes))
    model.add(Activation("softmax"))
    
    # return the constructed network architecture
    return model
    
dataset = './/images'
# initialize the data and labels
print("[INFO] loading images...")
data = []
labels = []
 
# grab the image paths and randomly shuffle them
imagePaths = sorted(list(paths.list_images(dataset)))
random.seed(42)
random.shuffle(imagePaths)
# loop over the input images
for imagePath in imagePaths:
    # load the image, pre-process it, and store it in the data list
    image = cv2.imread(imagePath)
    image = cv2.resize(image, (image_size, image_size))
    #image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    image = img_to_array(image)
    data.append(image)
    # extract the class label from the image path and update the
    # labels list
    label = imagePath.split(os.path.sep)[-2]
    #print(label)
    if label == 'empty':
        label = 0
    elif label == 'b_p':
        label = 1
    elif label == 'b_r':
        label = 2
    elif label == 'b_n':
        label = 3
    elif label == 'b_b':
        label = 4
    elif label == 'b_k':
        label = 5
    elif label == 'b_q':
        label = 6
    elif label == 'w_P':
        label = 7
    elif label == 'w_R':
        label = 8
    elif label == 'w_N':
        label = 9
    elif label == 'w_B':
        label = 10
    elif label == 'w_K':
        label = 11
    elif label == 'w_Q':
        label = 12
    else:
        raise ValueError('Unknown class: %s!' % label)
    labels.append(label)
        
# scale the raw pixel intensities to the range [0, 1]
data = np.array(data, dtype="float") / 255.0
labels = np.array(labels)
 
# partition the data into training and testing splits using 75% of
# the data for training and the remaining 25% for testing
(trainX, testX, trainY, testY) = train_test_split(data,
    labels, test_size=0.25, random_state=42)# convert the labels from integers to vectors
trainY = to_categorical(trainY, num_classes=13)
testY = to_categorical(testY, num_classes=13)

#trainX = trainX.reshape(-1, 200, 200, 3)

# initialize the number of epochs to train for, initial learning rate,
# and batch size
EPOCHS = 20
INIT_LR = 1e-3
BS = 32# initialize the model
print("[INFO] compiling model...")
model = LeNet.build(width=image_size, height=image_size, depth=3, classes=13)
opt = Adam(lr=INIT_LR, decay=INIT_LR / EPOCHS)
model.compile(loss="binary_crossentropy", optimizer=opt,
    metrics=["accuracy"])
 
# train the network
print("[INFO] training network...")
history = model.fit(trainX, trainY, batch_size=BS,
    validation_data=(testX, testY),# steps_per_epoch=len(trainX) // BS,
    epochs=EPOCHS, verbose=1)
 
# save the model to disk
print("[INFO] serializing network...")
model.save("chess-detector")

#print(np.sum([np.prod(v.get_shape().as_list()) for v in tf.trainable_variables()]))
model.summary()

plt.xlabel('Epoch Number')
plt.ylabel("Loss / Accuracy Magnitude")
plt.plot(history.history['loss'], label="loss")
plt.plot(history.history['accuracy'], label="acc")
plt.plot(history.history['val_loss'], label="val_loss")
plt.plot(history.history['val_accuracy'], label="val_acc")
plt.legend()
plt.show()
