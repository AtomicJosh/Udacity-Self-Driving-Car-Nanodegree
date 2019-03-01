# Advanced Lane Finding Project
Udacity Self-Driving Car Engineer Nanodegree Program

---

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/chess.png "Undistorted"
[image2]: ./output_images/road_undistort.png "Road Transformed"
[image3]: ./output_images/warper.png "Warper"
[image4]: ./output_images/binary.png "Binary"
[image5]: ./output_images/color_fit_lines.jpg "Fit Visual"
[image6]: ./output_images/polyfit.png "Polyfit"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the third code cell of the IPython notebook located in "Advanced_Lane_lines.ipynb".  

I started by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function and obtained this result:

![alt text][image1]

I applied this distortion correction to the test image using the `cv2.undistort()` function

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

By calculating the camera calibration matrix and distortion coefficients, I used distortion correction to undistort the test_images as shown here:

![alt text][image2]

This step can be found in the fourth cell of Advanced-Lane-Lines.ipynb.

#### 2. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warper()`, which appears in the sixth cell of `Advanced_Lane_Lines.ipynb`.  The `warper()` function takes as inputs an image (`img`) and its index. Inside the function, it computes the image's source (`src`) and destination (`dst`) points.  To increase the flexibility of the code, I chose not to hardcode the source and destination points, but to instead rely on percentage sizes of the images in the following manner:

```python
src = np.float32([
    [undist_img.shape[1] * 0.40, undist_img.shape[0] * 0.67],
    [undist_img.shape[1] * 0.61, undist_img.shape[0] * 0.67],
    [undist_img.shape[1] * 1.00, undist_img.shape[0] * 1.00],
    [undist_img.shape[1] * 0.04, undist_img.shape[0] * 1.00]])

dst = np.float32([
    [undist_img.shape[1] * 0.00, undist_img.shape[0] * 0.00],
    [undist_img.shape[1] * 1.00, undist_img.shape[0] * 0.00],
    [undist_img.shape[1] * 1.00, undist_img.shape[0] * 1.00],
    [undist_img.shape[1] * 0.00, undist_img.shape[0] * 1.00]])
```

This resulted in the following source and destination points:

| Source     | Destination  |
|:----------:|:------------:|
| 512,  482  | 0,      0    |
| 781,  482  | 1280,   0    |
| 1280, 720  | 1280, 720    |
| 51,   720  | 0,    720    |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image3]

#### 3. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps in the eighth cell in `Advanced_Lane_Lines.ipynb`.  I used the saturation channel from the HLS color space due to picking up the white and yellow lines well. Unfortunately, it is poor at picking up shadows. To compensate for this, I used the L channel of the LUV color space, which identifies white lane lines well, but it misses the yellow lane lines. To capture the yellow lane lines, I used the B channel of the LAB color space. Coincidentally, the B channel is also poor at identifying the white lane lines. Here's an example of my output for this step.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Using the binary image from the previous step, I identified the peaks in a histogram which is indicative of the lane lines in the image. Using the numpy nonzero function, all nonzero pixels were identified, then the polynomials were fit to each of the lane lines using numpy's polyfit function. This can be found in code cell ten of `Advanced_Lane_Lines.ipynb`.

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

With the lane lines identified, I was able to determine how far from center of the lane the vehicle was. The average x-intercept from each lane line was determined from the polynomials. I calculated the distance from the center of the lane by taking the absolute value of the vehicle's position and by halving the horizontal axis and determining the difference between the two. The distance judged by pixels was determined by measuring the length of the lane lines and determining the measurement in meters could be determined by multiplying the pixels by 0.005285714. This can be found in
code cell 10 in `Advanced_Lane_Lines.ipynb'.

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in code cell 10 in `Advanced_Lane_Lines.ipynb'.  Here is an example of my result on a test image:

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./output_videos/project_video_output.mp4)
---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

I was optimistic about how my approach would work considering the using the three color channels for lane detection and robustness to shadows. My approach worked well in the required project video, and moderately well in the challenge video. It did quite poorly in the harder challenge video with the tight turns. Given more time, I would have tried to find out how to handle these sharp turns better.
