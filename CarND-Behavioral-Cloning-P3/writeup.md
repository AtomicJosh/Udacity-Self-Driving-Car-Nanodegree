# **Behavioral Cloning**

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/ex1_center.jpg "Center Driving"
[image2]: ./examples/ex2_recover1.jpg "Recovery Image"
[image3]: ./examples/ex3_recover2.jpg "Recovery Image"
[image4]: ./examples/ex4_recover3.jog "Recovery Image"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network
* writeup_report.md or writeup_report.pdf summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing
```sh
python drive.py model.h5
```
Note: I couldn't get the Keras generator working with the CarND environment provided by Udacity. Instead, this model was made using Keras version 2.1.5 and Tensorflow version 1.6.0. These versions will be needed to run the model in the simulator.

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

I used the nVidia Autonomous Car Group model as the basis for my design. I used three epochs with a batch size of 32. See section "Solution Design Approach" below for a more detailed discussion of the architecture.

#### 2. Attempts to reduce overfitting in the model

I didn't incorporate any max pooling nor dropout layers to reduce overfitting. Instead, I relied on having a lot of data and using just three epochs.

The model was trained (80%) and validated (20%) on different data sets to ensure that the model was not overfitting (model.py line 111). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 121).

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used the Udacity provided driving data which had the car driving down the center of the road for a lap. I then collected my own data of pulling off to the far side of the road and recording driving back to the center of the road. I read that using the mouse rather than the keyboard provided better results, so I used the mouse for all steering. I gathered this recovery data from the sides over the course of three laps, equally from both sides of the road.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

I started with the nVidia Autonomous Car Group model as my basis. I then played with the epochs and batch sizes and settles upon 3 epochs with a batch size of 32. My logic for choosing the three epochs is to minimize the chances of overfitting. A side benefit of such low epochs is that training the model was time intensive. Training on a CPU with no GPU for each epoch took 12 hours. Training on an nVidia GTX 1080 Ti GPU took half an hour for each epoch.

Finally, I tested the car around the track in the simulator. It never touched a lane line, however it did get close on one corner. Thus, it successfully drove autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture (model.py lines 93-104) consisted of the nVidia Autonomous Car Group model with the following layers and layer sizes:

| Layer | Output Shape | Parameters |
|:-----:|:------------:|:----------:|
| Lambda | (None, 160, 320, 3) | 0 |
| Cropping2D | (None, 90, 320, 3) | 0 |
| Conv2D | (None, 43, 158, 24) | 1824 |
| Conv2D | (None, 20, 77, 36) | 21636 |
| Conv2D | (None, 8, 37, 48) | 43248 |
| Conv2D | (None, 6, 35, 64) | 27712 |
| Conv2D | (None, 4, 33, 64) | 36928 |
| Flatten | (None, 8448) | 0 |
| Dense	| (None, 100) | 844900 |
| Dense	|	(None, 50) | 5050 |
| Dense | (None, 10) | 510 |
| Dense	| (None, 1) | 11 |

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first used the Udacity provided data which recorded a lap on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image1]

I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to recover from the edges of the road. These images show what a recovery looks like:

![alt text][image2]
![alt text][image3]
![alt text][image4]

To augment the data sat, I also flipped images and angles thinking that this would help generalize the data (model.py lines 76-77) for turning in both directions.

After the collection process, I had 42,405 images. I then preprocessed this data by regularizing the pixel values to 0 +/- 0.5 (model.py line 86). As recommended in the course lectures, I also cropped the data to only focus on the area of the images that had road data (model.py line 87).

I finally randomly shuffled the data set and put 20% of the data into a validation set.

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 3 as evidenced by the car successfully driving around the track without touching a lane line. I used an adam optimizer so that manually training the learning rate wasn't necessary.
