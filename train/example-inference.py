#!/usr/bin/env python3

import time
import cv2
import numpy as np
import math
import os

from keras.preprocessing.image import img_to_array
from keras.models import load_model
#from keras.backend import set_session
import tensorflow.compat.v1 as tf
import os, sys, imutils, argparse

model_path = "chess-detector/"
image_path = "test/"
image_size = 100

model = load_model(model_path, compile = True)

files = os.listdir(image_path)


for file_name in sorted(files):
    print("="*40)
    print(file_name)
    image = cv2.imread(image_path + file_name)
    image = cv2.resize(image, (image_size, image_size))
    image = img_to_array(image)
    image = np.array(image, dtype="float") / 255.0
    image = image.reshape(-1, image_size, image_size, 3)
    
    start_time = time.perf_counter()
    prediction = np.argmax(model.predict(image))
    print(time.perf_counter()-start_time)

    if prediction == 0:
        print("Empty")
    elif prediction == 1:
        print("Black pawn")
    elif prediction == 2:
        print("Black rook")
    elif prediction == 3:
        print("Black knight")
    elif prediction == 4:
        print("Black bishop")
    elif prediction == 5:
        print("Black king")
    elif prediction == 6:
        print("Black queen")
    elif prediction == 7:
        print("White pawn")
    elif prediction == 8:
        print("White rook")
    elif prediction == 9:
        print("White knight")
    elif prediction == 10:
        print("White bishop")
    elif prediction == 11:
        print("White king")
    elif prediction == 12:
        print("White queen")


