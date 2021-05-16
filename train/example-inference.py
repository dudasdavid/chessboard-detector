#!/usr/bin/env python3

import time
import cv2
import numpy as np
import os

from keras.preprocessing.image import img_to_array
from keras.models import load_model
import h5py
from keras import __version__ as keras_version
import os
import helper_lib

model_path = "model.best.h5"
image_path = "test/"
image_size = 100
padding_left = 50
padding_right = 50
padding_top = 50
padding_bottom = 50
# margin has to be less or equal to padding_left or padding_top
square_margin = 50

f = h5py.File(model_path, mode='r')
model_version = f.attrs.get('keras_version')
keras_version = str(keras_version).encode('utf8')

if model_version != keras_version:
    print('You are using Keras version ', keras_version,
            ', but the model was built using ', model_version)

model = load_model(model_path)

files = os.listdir(image_path)

for file_name in sorted(files):

    print("="*40)
    print(file_name)
    image = cv2.imread(image_path + file_name)

    rows,cols = image.shape[:2]
    row_height = int((rows - padding_top - padding_bottom) / 8)
    col_width = int((cols - padding_left - padding_right) / 8)

    result_frame = image.copy()

    start_time = time.perf_counter()
    for i in range(0,8):
        for j in range(0,8):
            square = image[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
            square = cv2.resize(square, (image_size, image_size))
            square = img_to_array(square)
            square = np.array(square, dtype="float") / 255.0

            prediction = np.argmax(model.predict(square[None, :, :, :], batch_size=1))
            label, short = helper_lib.class2label(prediction)
            print(label, short)

            cv2.putText(result_frame, label,
                    (padding_top + j * row_height + 25, padding_left + i * col_width + 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

    print(time.perf_counter()-start_time)
    cv2.imwrite(image_path + file_name[:-4] + "_result.jpg", result_frame)




