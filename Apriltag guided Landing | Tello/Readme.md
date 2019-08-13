# Apriltag guided landing
Here is an algorithm for guided precesion landing of a micro drone like DJI Tello.

## Why Apriltag ?
Apriltags are good tool in robotics and are used to localize the camera or the tag itself.
## How is Location obtained ?
Well, now it gets little theoretical. Following are the steps to obtain the location:


Well now it gets little theoretical. Following are the steps to optain the location:-
1) Apriltag python library apriltag.py provides us with the homography of the tag.
```
detector = apriltag.Detector()
result = detector.detect(gray)
H = np.array(result[0].homography)
```
2) Once having the homography, we can obtain the localization in terms of rotation and translation matrix [R|t] by simple maths.
```
H = np.array(result[0].homography)      
h1h2h3 = np.matmul(K_inv,H) # K_inv is the inverse of the camera matrix

h1T = h1h2h3[:,0]
h2T = h1h2h3[:,1]
h3T = h1h2h3[:,2]

h1Xh2T = np.cross(h1T,h2T)

h1_h2_h1Xh2T = np.array([h1T,h2T,h1Xh2T])
h1_h2_h1Xh2 = np.transpose(h1_h2_h1Xh2T)

u, s, vh = np.linalg.svd(h1_h2_h1Xh2, full_matrices=True)

uvh = np.matmul(u,vh)
det_OF_uvh = np.linalg.det(uvh)

M = np.array([[1,0,0],[0,1,0],[0,0,det_OF_uvh]])

T = h3T/np.linalg.norm(h1T) # Translation Matrix
r = np.matmul(u,M)
R = np.matmul(r,vh) # Rotation matrix

```
Soon I will also provide a resource to read about this calculation 
3) The R and t you got above are of the tag with respect to the camera. But to localize a robot, you need the location of the robot with respect to the tag. Thus the inverse of  [R|t] which is [R transpose | -1*Rtranspose*t].
```
T_t = np.reshape(T,(3,1)) #taking the taking of column matrix
neg_Rt_T = -1*np.dot(R.T,T_t)
```
4) Finally here you have the position of the robot with respect to the tag.

## Landing the tello
Once we have the position of the drone, as explained above, we can now use the tag as the landing pad.

### Prerequisites and Installing
You need to have the followings installed on your system.
1) Opencv :- to view and process the image
```
Google it xD
```
2) Apriltag.py :- to get the data od apriltag
```
https://pypi.org/project/apriltag/
```
3) djitellopy :- to be able to control the DJI Tello drone.
```
https://github.com/Mohit505Git/DJITelloPy/blob/master/djitellopy/tello.py
```

# Fun fact

Tello does not have an accessible downward-facing camera, So, I mounted a small mirror at 45 degrees using a small styrofoam attachment on the drone.

That's all
Hope you like it!

Have a nice day.
## Author

* **Mohit Singh**


