#!/usr/bin/env python3.8

import time
import cv2
import numpy as np
import os

from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
import h5py
from tensorflow.keras import __version__ as keras_version
import os
import helper_lib
from argparse import ArgumentParser

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

def build_argparser():
    # parse command line arguments.
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=False, type=str, default="model.best.h5",
                        help="Path to model")         

    return parser

# grab command line args
args = build_argparser().parse_args()

model_path = args.input
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
    print("Open file: %s" % file_name)
    if "_result" in file_name:
        print("Skipping...")
        continue

    image = cv2.imread(image_path + file_name)

    rows,cols = image.shape[:2]
    row_height = int((rows - padding_top - padding_bottom) / 8)
    col_width = int((cols - padding_left - padding_right) / 8)

    result_frame = image.copy()

    squares = []
    for i in range(0,8):
        for j in range(0,8):
            square = image[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
            square = cv2.resize(square, (image_size, image_size))
            square = img_to_array(square)
            square = np.array(square, dtype="float") / 255.0
            squares.append(square)


    start_time = time.perf_counter()
    predictions = model.predict_on_batch(np.asarray(squares))
    j = -1
    fen_input = []
    for i, pred_i in enumerate(predictions):
        if i % 8 == 0:
            j+=1

        prediction = np.argmax(pred_i)
        label, short = helper_lib.class2label(prediction)
        #print(label, short)
        fen_input.append(short)

        cv2.putText(result_frame, label,
                (80 + (i % 8) * row_height, 80 + j * col_width),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2)

    print("Batch inference time: %.3f" % (time.perf_counter()-start_time))
    #print(fen_input)
    fen = helper_lib.get_fen(fen_input)
    print("Fen: %s" % fen)
    result_name = file_name[:-4] + "_result.jpg"
    print("Result saved to %s" % result_name)
    cv2.imwrite(image_path + result_name, result_frame)



