### Processes of Automotive Software Development - Lane Detection Feature

Goal of this project is to have a software pipeline to identify and track the position of lane lines in front of the vehicle based on raw camera data from CARLA Simulator. Useful features for identifying lane lines are: color, shape, orientation, position and etc. 
Basic pipeline can consist of color space transformation, grayscaling, then region masking, then finding edges, and finally using a Hough Transform to identify line segments

<p align="center">
  <img src="https://user-images.githubusercontent.com/45635888/151385644-8a1b0a51-2e30-4947-b969-f258ae0a7a75.png">
</p>

<p align="center">
  Real Time image processing(Left, proceed image/Right, original image)
</p>



