---
layout: post
title: Kinect and projector pair calibration
---

## Introduction
Sometimes we want to combine Microsoft Kinect and a projector to create cool [Augmented Reality (AR) applications](http://genekogan.com/works/kinect-projector-toolkit/). Existing methods, such as [RGBDdemo][1] and [KinectProjectorToolkit][2] either requires printed checkerboard patterns or a large room to calibrate Kinect depth/color cameras and a projector. 

In most simple AR applications, we bind the Kinect and the projector (their FOV overlaps), thus the relative rotation and translation between the two are fixed. In this case, neither a printed checkerboard pattern or a large room is needed. We can calibrate the system by projecting a checkerboard pattern to a white flat wall,  then move the whole Kinect-projector pair together to capture at least 3 shots from different position/orientations for [Zhang's method][5].

In this article, we focus on calibrating the **intrinsic parameters of the projector** and the **extrinsic parameters between the projector and the Kinect depth camera**. The calibration data of the Kinect color camera and depth camera can either be obtained from Kinect Windows SDK or calibrated using [Zhang's method][5], thus they are not discussed in this article.

In the following paragraphs we are going to talk about this simple calibration method.

## Projected checkerboard image
We first generate a checkerboard image pattern in OpenCV, where `boardSize` contains the number of squares in row and column, `cbPts2d` stores a list of inner corners of the checkerboard is given by:
$$ \mathbf{P}^{2d}_{p} = \[ \mathbf{q}_0, \mathbf{q}_1,\dots \mathbf{q}_i, \dots \mathbf{q}_N \] $$, where \\(\mathbf{q}_i = \[ u_i, v_i \] \\) is the 2D coordinate of the *ith* checkerboard corner in projector image space, *N = `boardSize.width`\*`boardSize.height`*.


```C++
Mat generateCheckerboardImg(Size imgSize, Size boardSize, vector& cbPts2d) {	
	int offset = 50; // opencv requires white boarders around checkerboard pattern

	// checkerboard image
	Mat imgCheckerboard(imgSize, CV_8UC3, Scalar::all(255));

	// block size
	int squareWidth = floor((imgSize.width - 2 * offset) / boardSize.width);
	int squareHeight = floor((imgSize.height - 2 * offset) / boardSize.height);

	// block color
	unsigned char color = 1;

	//! The order must be consistent with OpenCV order: row first then column, each row sweep from left to right
	for (int y = offset; y < imgSize.height - offset; y = y + squareHeight) {
		color = ~color;
		if (y + squareHeight > imgSize.height - offset) {
			break;
		}

		for (int x = offset; x < imgSize.width - offset; x = x + squareWidth) {
			color = ~color;
			if (x + squareWidth > imgSize.width - offset) {
				break;
			}

			// save checkerboard points
			if (x > offset && y > offset) {
				cbPts2d.push_back(Point2f(x, y));
			}

			// color the block
			Mat block = imgCheckerboard(Rect(x, y, squareWidth, squareHeight));
			block.setTo(Scalar::all(color));

		}
	}

	return imgCheckerboard;
}
```
The code is inspired by [Haris][3].

Then we project this image to a white flat wall using the projector. In the meanwhile, we using the Kinect to capture a depth and a color frame of the projected checkerboard pattern. 

## Getting the 3D-2D coordinates of the checkerboard corners
Let \\(\mathbf{P}^{3d}\\) be a set of 3D locations of the projected checkerboard corners:
$$ \mathbf{P}^{3d} = \[ \mathbf{x}_0, \mathbf{x}_1,\dots \mathbf{x}_i, \dots \mathbf{x}_N \] $$, where \\(\mathbf{x}_i = \[ X_i, Y_i, Z_i \] \\) is the 3D coordinate of the *ith* checkerboard corner in Kinect depth camera view space.

We extract checkerboard corners \\(\mathbf{P}^{2d}_{c}\\) from Kinect color image using [findChessboardCorners][6] and their corresponding 3D locations \\(\mathbf{P}^{3d}\\) from Kinect depth image. I'll skip the details of this part, since this is very simple if you use Kinect Windows SDK v2.0. For more information please refer to [Kinect CoordinateMapper][4]. Note \\(\mathbf{P}^{2d}_{c}\\) is only used to extract \\(\mathbf{P}^{3d}\\) from depth image using [Kinect CoordinateMapper][4], but if you want to calibrate Kinect color camera keep  \\(\mathbf{P}^{2d}_{c}\\) for later use.

Now we have the 3D-2D point pairs (\\(\mathbf{P}^{3d}\\) and \\(\mathbf{P}^{2d}_{p}\\)) to calibrate the projector intrinsics. But if you send the point pairs directly to OpenCV's [calibrateCamera][5] an exception will be raised, because this function requires the Z values of `objectPoints` to be zeros, since [Zhang's method][5] assumes all `objectPoints` reside on the XY plane in checkerboard object space and thus the Z values are 0, then the 3x4 projection matrix **K[R|T]**  is reduce to a 3x3 homography **H**. 

If we plot \\(\mathbf{P}^{3d}\\) we can see that they reside on the same plane but the Z values are nonzero, because their 3D coordinates are defined in Kinect depth camera view space rather than checkerboard object space. 


One may ask **can we generate the 3D coordinates of these checkerboard corners like what we do to the printed checkerboard?** The answer is no, unlike a real checkerboard, the projected image is distorted and skewed due to the perspective projection each time we change the projector's position/orientation in respect to the wall. So each projected checkerboard image on the wall has different unknown scales and geometries.

## Rotate 3D points using eigenvectors
One workaround is to estimate a rotation and translation between Kinect depth camera's view space and the checkerboard object space, then rotate and translate \\(\mathbf{P}^{3d}\\) to the canonical view, so that they reside in the XY plane of Kinect depth camera view space. This requires the parameters of the checkerboard plane, since we know \\(\mathbf{P}^{3d}\\) form a planar shape with arbitrary orientations and translations, we can estimate the plane by the following methods:

1. choose any **three non-collinear** points to calculate the plane's normal (Z) and X, Y axis directions in the checkerboard object space.
2. use **all** the points in to fit a plane subject to minimizing the linear least squares error. Then choose any two points to calculate X(or Y) axis and the other axis is the cross product of normal and X(or Y): `Y = cross(X, Z)`.
3. use the eigenvectors of \\(\mathbf{P}^{3d}\\)'s covariance matrix as XYZ axes.

If we observe option 1, which three points should we choose to estimate the plane? The same question applies to option 2 too, which two points should we use to estimate X (or Y) axis? Statistically, we prefer option 3 since it makes use of the distribution of \\(\mathbf{P}^{3d}\\).

### Geometric interpretation of eigenvectors and Singular Value Decomposition (SVD)

If we draw the eigenvectors of \\(\mathbf{P}^{3d}\\)'s covariance matrix, what are the directions of the three eigenvectors? The first two must be on the checkerboard plane and the third is the normal of the plane! Let us recall **eigen decomposition**: 

the covariance matrix of \\(\mathbf{P}^{3d}\\) is given by:
$$ \mathbf{\Sigma} = (\mathbf{x}_i - \bar{\mathbf{x}})(\mathbf{x}_i - \bar{\mathbf{x}})^{T} $$, where \\(\mathbf{x}_i\\) is the *ith* point, and \\(\mathbf{P}^{3d}\\) is the mean of \\(\mathbf{P}^{3d}\\). 

The eigen decomposition of \\(\mathbf{\Sigma}\\) is given by:
$$ \mathbf{\Sigma} = \mathbf{U}\mathbf{S}^{2}\mathbf{U}^T $$, where \\(\mathbf{U}\\)'s columns are the eigenvectors of \\(\mathbf{\Sigma}\\) and **they are also the left singular vectors of \\( (\mathbf{x}_i - \bar{\mathbf{x}}) \\)**. 

So we use SVD to obtain eigenvectors instead of computing an expensive covariance matrix:
1. Center \\(\bar{\mathbf{P}}^{3d}\\) at origin by:
$$ \bar{\mathbf{P}}^{3d} = \mathbf{P}^{3d} - mean(\mathbf{P}^{3d}) $$
2. Apply SVD to \\(\bar{\mathbf{P}}^{3d}\\):
$$ \bar{\mathbf{P}}^{3d} = \mathbf{U}\mathbf{S}\mathbf{V}^T $$. 
3. To rotate \\(\bar{\mathbf{P}}^{3d} \\) to Kinect depth camera view space's XY plane, we only need to left multiply the inverse of \\(\mathbf{U}\\), i.e.,  \\(\mathbf{U}^{T}\\) to \\(\bar{\mathbf{P}}^{3d} \\):
$$ \mathbf{P}^{3d}_{obj} = \mathbf{U}^{T}\bar{\mathbf{P}}^{3d} $$.
4. Due to projector and Kinect sensor noise and nonplanarity of the wall, the translated and rotated \\(\mathbf{P}^{3d}_{obj}\\) may have very small Z values, we can orthogonal project the points to depth camera view space XY plane by manually setting Z = 0.

Note in step 3, we do not constrain the sign of eigenvectors in this article because they do not affect projector **intrinsics** calibration.

Finally, we have 3D checkerboard corners  \\(\mathbf{P}^{3d}_{obj}\\)  and 2D checkerboard corners in projector image space \\(\mathbf{P}^{2d}_{p}\\) to calibrate the projector intrinsics using [calibrateCamera][5].


[1]:http://rgbdemo.org/index.php/Documentation/TutorialProjectorKinectCalibration
[2]:https://github.com/genekogan/KinectProjectorToolkit
[3]:http://answers.opencv.org/question/27917/how-to-create-a-chess-board/?answer=27943#post-id-27943
[4]:https://msdn.microsoft.com/en-us/library/windowspreview.kinect.coordinatemapper.aspx
[5]:https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera
[6]:https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findchessboardcorners