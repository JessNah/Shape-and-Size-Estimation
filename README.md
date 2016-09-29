# Kinect---Shape-and-Size-Detection
Uses the Kinect for Windows v2 API and camera to detect if an object is placed in view of the camera(Only detects large pixel clusters to eliminate noise fault detections) and once the object is placed in view, a black and white image of the object's shape is saved plus, it's width/height dimension estimations are calculated and displayed using a formula I created 



<img src = "https://cloud.githubusercontent.com/assets/14356838/18961113/dfd2ecae-8639-11e6-938a-50c44879a395.png">

<img src = "https://cloud.githubusercontent.com/assets/14356838/18961112/dfd300d6-8639-11e6-9556-862d5e4c8c0e.png">

I ran the code for the above images on the same box, while keeping it horizontally and vertically for each test.
As can be seen by the height and width dimensions in the images, the program I've written seems to give relatively
precise size estimations for the object as the height and widths match up.

The output size information can be seen printed on the console as well as the image that the program prints.
The grey lines display the average straight edge that was used when the program calculated the dimensions.


Below are more images showing examples of the code's ability to detect a new object in front of it, and extract only that object's shape whilsty neglecting the surroundings along with its capabilities to detect and approximate the object's horizontal and vertical edges which was used for the purposes of size estimation.

<img src="https://cloud.githubusercontent.com/assets/14356838/18962680/d11e26be-863f-11e6-82bd-daf071dd37f0.png">
<img src="https://cloud.githubusercontent.com/assets/14356838/18962693/e0a4c048-863f-11e6-8c7c-f7ea00e0abde.png">
<img src="https://cloud.githubusercontent.com/assets/14356838/18962686/d9745f04-863f-11e6-8e81-a4669a96563b.png">

This code was built upon a generic kinect template provided by microsoft.
Copyright of the template is of Microsoft's.
