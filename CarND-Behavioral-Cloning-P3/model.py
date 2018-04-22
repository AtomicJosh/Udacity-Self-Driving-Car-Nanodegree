# Note: keras version 2.1.5 and tensorflow version 1.6.0 were used in the development of this script

import cv2
import csv
import numpy as np
import os
import sklearn
from sklearn.model_selection import train_test_split
from keras.models import Sequential, Model
from keras.layers import Flatten, Dense, Lambda, Conv2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
import matplotlib.pyplot as plt

batch_size=32

def getDrivingLog(path, skipHeader=False):
    lines = []
    with open(path + '/driving_log.csv') as csvFile:
        reader = csv.reader(csvFile)
        if skipHeader: next(reader, None)
        for line in reader: lines.append(line)
    return lines

def getImage(path):
    directories = list(filter(lambda directory: os.path.isfile(directory + '/driving_log.csv'), [x[0] for x in os.walk(path)]))
    centerTotal = []
    leftTotal = []
    rightTotal = []
    angleTotal = []
    
    for directory in directories:
        lines = getDrivingLog(directory)
        center = []
        left = []
        right = []
        angles = []
        for line in lines[1:]:
            angles.append(float(line[3]))
            center.append(directory + '/' + line[0].strip())
            left.append(directory + '/' + line[1].strip())
            right.append(directory + '/' + line[2].strip())
        centerTotal.extend(center)
        leftTotal.extend(left)
        rightTotal.extend(right)
        angleTotal.extend(angles)

    return (centerTotal, leftTotal, rightTotal, angleTotal)

def combineImages(center, left, right, angle, correction):
    img = []
    img.extend(center)
    img.extend(left)
    img.extend(right)
    angles = []
    angles.extend(angle)
    angles.extend([x + correction for x in angle])
    angles.extend([x - correction for x in angle])
    return (img, angles)

def generator(samples, batch_size=batch_size):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        samples = sklearn.utils.shuffle(samples)
        
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for imagePath, angle in batch_samples:
                originalImage = cv2.imread(imagePath)
                image = cv2.cvtColor(originalImage, cv2.COLOR_BGR2RGB)
                images.append(image)
                angles.append(angle)
                # Flipping
                images.append(cv2.flip(image,1))
                angles.append(angle*-1.0)

            # crop to only see the road
            inputs = np.array(images)
            outputs = np.array(angles)
            yield sklearn.utils.shuffle(inputs, outputs)

def createPreProcessingLayers():
    model = Sequential()
    model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
    model.add(Cropping2D(cropping=((50,20), (0,0))))
    return model

def NVIDIAModel():
    # NVIDIA Autonomous Car Group model
    
    model = createPreProcessingLayers()
    model.add(Conv2D(24, (5,5), strides=(2,2), activation='relu'))
    model.add(Conv2D(36, (5,5), strides=(2,2), activation='relu'))
    model.add(Conv2D(48, (5,5), strides=(2,2), activation='relu'))
    model.add(Conv2D(64, (3,3), activation='relu'))
    model.add(Conv2D(64, (3,3), activation='relu'))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))
    return model

centerPaths, leftPaths, rightPaths, angs = getImage('data')
imagePaths, angs = combineImages(centerPaths, leftPaths, rightPaths, angs, 0.2)
print('Total Images: {}'.format(len(imagePaths)))

samples = list(zip(imagePaths, angs))
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

print('Train samples: {}'.format(len(train_samples)))
print('Validation samples: {}'.format(len(validation_samples)))

train_generator = generator(train_samples, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)

model = NVIDIAModel()

model.compile(loss='mse', optimizer='adam')
history_object = model.fit_generator(train_generator, 
                                     steps_per_epoch=len(train_samples),
                                     validation_data=validation_generator,
                                     validation_steps=len(validation_samples), 
                                     epochs=3,
                                     max_queue_size=1,
                                     verbose=1)

model.save('model.h5')
print(history_object.history.keys())
print('Loss')
print(history_object.history['loss'])
print('Validation Loss')
print(history_object.history['val_loss'])

plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()
