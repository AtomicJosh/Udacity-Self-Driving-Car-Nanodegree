# **Traffic Sign Recognition**
Udacity Self-Driving Car Engineer Nanodegree Program

---

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: a1.jpg "Yield Sign"
[image2]: a2.jpg "Road Work"
[image3]: a3.jpg "Turn Right Ahead"
[image4]: a4.png "No Vehicles"
[image5]: a5.png "Priority Road"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! Here is a link to my [project code](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb)

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the pandas library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34799.
* The size of test set is 12630.
* The shape of a traffic sign image is (32, 32, 3).
* The number of unique classes/labels in the data set is 43.

#### 2. Include an exploratory visualization of the dataset.

The dataset was explored to see what the images looked like and their corresponding names. See cell 4 of the notebook to see this visualized.

![alt text][image1]

### Design and Test a Model Architecture

#### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc.

First, I converted the images to grayscale using OpenCV's COLOR_RGB2GRAY function.
Second, I normalized all of the images as discussed in the TensorFlow lectures to reduce the disparity within the data.
Finally, the images were reshaped to get rid of the RGB values.

#### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model used the LeNet-5 architecture from the previous assignment with an added Dropout layer in layers 3 and 4:

| Layer | Description	|
|:-----:|:-----------:|
| Input | 32x32x1 grayscale image |
| Convolution 3x3 | 1x1 stride, valid padding, outputs 28x28x6 |
| RELU |  |
| Max pooling	| 2x2 stride,  outputs 14x14x6 |
| Convolution 3x3 | 1x1 stride, valid padding, outputs 10x10x16 |
| RELU |  |
| Max pooling	| 2x2 stride,  outputs 5x5x16 |
| Flatten | outputs 400 |
| Fully connected	| input 400, outputs 120 |
| RELU	|	 |
| Dropout | keep probability 0.5 |
| Fully connected	| input 120, outputs 84 |
| RELU	|	 |
| Dropout | keep probability 0.5 |
| Fully connected	| input 84, outputs 43 |

#### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

I used the hyperparameters from the previous LeNet assignment as a starting point. I used an AdamOptimizer with a learning rate of 0.001. I used the original epochs of 20 with a batch size of 128 but couldn't meet the required 0.93 validation set accuracy. I changed the values iteratively until ultimately arriving at 50 epochs with a batch size of 64 giving a validation set accuracy of 0.963.

#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* validation set accuracy of 0.963 (see cell 10)
* test set accuracy of 0.944 (see cell 11)

I had a lot of difficulties with this assignment. Originally, I implemented GoogLeNet (http://arxiv.org/abs/1409.4842), but I had a number of problems with trying to get it to work. As I was running out of time with this assignment, I ultimately went back to the LeNet design used in the previous LeNet assignment.

If I had more time, I'd try to get GoogLeNet working. I also would like to spend more time on the preprocessing. I also didn't do data augmentation as I originally planned due to time constraints.

### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

The model was quite accurate with predicting four of the 5 images correctly. The image it predicted wrong was the turn right ahead sign. This could be due to the training set having few examples to train from and could have been solved with data augmentation.

#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set.

Here are the results of the prediction:

| Image | Prediction |
|:-----:|:----------:|
| Yield | Yield |
| Road work | Road work |
| Turn right ahead | Roundabout mandatory |
| No vehicles | No Vehicles |
| Priority road | Priority road |

The model was able to correctly guess 4 of the 5 traffic signs, which gives an accuracy of 80%. This is quite a bit worse than the validation and test set accuracy. The dataset was a bit skewed with some classes having far more images than others. This could have been alleviated with data augmentation. I think this is a large contributing factor to the poor performance of the random internet Gernman traffic signs.

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 11th cell of the Ipython notebook.

![alt text][image1]

For the first image, the model is absolutely certain this is a yield sign with a probability of 1.0, which is correct. The top five soft max probabilities were:

| Probability | Prediction |
|:-----------:|:----------:|
| 1.0 | Yield |
| 0.0 | Road work |
| 0.0	| No vehicles |
| 0.0	| Priority road |
| 0.0 | Ahead only |


![alt text][image2]

For the second image, the model is absolutely certain this is a road work sign with a probability of 1.0, which is correct. The top five soft max probabilities were:

| Probability | Prediction |
|:-----------:|:----------:|
| 1.0 | Road work |
| 0.0 | Beware of ice/snow |
| 0.0 | Keep left |
| 0.0	| Dangerous curve to the right |
| 0.0 | Bicycles crossing |

![alt text][image3]

For the third image, the model thinks this is a roundabout mandatory sign, but it has a relatively low certainty at just 74.13%. This is actually a turn right ahead sign. Interestingly, the correct sign wasn't even in the top five predictions. This could be due to having few training examples of this class and could have been solved with data augmentation or finding more images of turn right ahead signs for the training set. The top five soft max probabilities were:

| Probability | Prediction |
|:-----------:|:----------:|
| 0.7413 | Roundabout mandatory |
| 0.2351 | Speed limit (100km/h) |
| 0.0167 | Priority road |
| 0.0017 | Speed limit (120km/h) |
| 0.0015 | Vehicles over 3.5 metric tons |

![alt text][image4]

For the fourth image, the model is absolutely certain this is a no vehicles sign with a probability of 1.0, which is correct. The top five soft max probabilities were:

| Probability | Prediction |
|:-----------:|:----------:|
| 1.0 | No vehicles |
| 0.0 | Priority road |
| 0.0	| Keep right |
| 0.0	| Speed limit (50km/h) |
| 0.0 | No passing |

![alt text][image5]

For the fifth image, the model is absolutely certain this is a priority road sign with a probability of 1.0, which is correct. The top five soft max probabilities were:

| Probability | Prediction |
|:-----------:|:----------:|
| 1.0 | Priority road |
| 0.0 | Yield |
| 0.0	| Roundabout mandatory |
| 0.0	| End of no passing by vehicles over 3.5 metric tons |
| 0.0 | Keep right |
