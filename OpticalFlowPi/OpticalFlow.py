"""
To do :- fast frame motion removal ... probably with the help of recursuion

Issues :- Time lag is experienced


"""
def locateFeatures(inImg):
    
    fast = cv2.FastFeatureDetector_create()# find and draw the keypoints
    fast.setThreshold(60)
    kp  = fast.detect(inImg,None)
    l=len(kp)
    a = np.empty((l,1,2),dtype = np.float32)
    for i in range(l):
        x = kp[i].pt[0]
        y = kp[i].pt[1]
        a[i] = [x,y]
    # print( "Threshold: {}".format(fast.getThreshold()) )
    # print( "nonmaxSuppression:{}".format(fast.getNonmaxSuppression()) )
    # print( "neighborhood: {}".format(fast.getType()) )
    # print( "Total Keypoints with nonmaxSuppression: {}".format(len(kp)) )
    return a

def get_distance():
    file = open('distance.txt','r')
    distance =file.read().split()
    #print distance
    file.close()
    if(len(distance)!= 0):
        return float(distance[0])
    else:
        time.sleep(0.01)
        return get_distance()
    

def imu():
    global old_angX , old_angY
    
    file = open('imu_angleXY.txt','r')
    ang = file.read().split()
    file.close()
    if(len(ang) != 0):
        angX = float(ang[0])
        angY = float(ang[1])
        dAngX = angX - old_angX
        dAngY = angY - old_angY
        old_angX = angX
        old_angY = angY
            #print("xdis",xdis)
            #print("AngX",dAngX)
            #print("AngY",dAngY)
           # print("Found",e)
        return dAngX, dAngY
    else:
        print("Not found",f)
        time.sleep(0.01)
        return imu()
            
        
def clacFlow(good_old,good_new):
    
    global x,y,Q
    
    transformation = cv2.estimateRigidTransform(good_old, good_new, False)
    
    if(transformation != None):
        #transformation = np.array([[1,1,1],[1,1,1]],dtype = int)
        scaling = np.sqrt(pow(transformation[0,0],2) + pow(transformation[0,1],2))  
        rotation = np.arctan2(transformation[1,0],transformation[0,0])
            
        translation = np.sqrt(pow(transformation[0,2],2) + pow(transformation[1,2],2))
        #retriving imu data
        dAngX, dAngY = imu()
        
        Q = Q + np.arctan2(transformation[1,0],transformation[0,0])
        x = x + transformation[0,2]
        y = y + transformation[1,2]
        #retriving the distance
        z = get_distance()
        #z = z + (1-scaling)
        return x,y,z,Q
    
    
    
    
#________________________________________Main Code Starts__________________
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 40
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.01)
 
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 15,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

#Create some random colors
#color = np.random.randint(0,255,(1000,3))
u = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    old_frame = frame.array
    rawCapture.truncate(0)
    u = u+1
    print("starting")
    if (u == 10 ):
        break
# Take first frame and find corners in it
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    old_frame = frame.array
    rawCapture.truncate(0)
    break
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = locateFeatures(old_gray)
print(p0)

# Create a mask image for drawing purposes
#mask = np.zeros_like(old_frame)

m=1
x,y,z_old,Q = 0,0,0,0
# variable for change in the angle 
old_angX = 0.0
old_angY = 0.0
angX = 0.0
angY = 0.0

# capture frames from the camera
e = 0
f = 0
xdis = 0

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
 
    frame_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.GaussianBlur(frame_gray,(5,5),0.3)
    #cv2.imshow("Frame",frame_gray)
    
    # calculate optical flow
    if(p0.size != 0):
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        # Select good points
        good_new = p1[st==1]
        good_old = p0[st==1]
        
        #print "good new",good_new
        #print "good old", good_old
    
        X,Y,Z,q = clacFlow(good_old,good_new)
        
        print("x",X*Z*0.01)# odometry x
        print("y",Y*Z*0.01)# odometry y
        print("z",Z)
        print("Q",q)
    else:
        rawCapture.truncate(0)
        continue
    print ("______________________________________-")
    # print("scaling",scaling,"translation",translation,"rotation",rotation)

    # draw the tracks
    #for i,(new,old) in enumerate(zip(good_new,good_old)):
    #     a,b = new.ravel()
    #     c,d = old.ravel()
    #     mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
    #     image = cv2.circle(image,(a,b),5,color[i].tolist(),-1)
    #img = cv2.add(image,mask)
    
    cv2.imshow('frame',image)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()

    if(m%10 == 0):
        p0 = locateFeatures(old_gray)
        # print "yes"

    else:
        p0 = good_new.reshape(-1,1,2)
    m=m+1
    # show the frame
    #cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
 
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
 
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

cv2.destroyAllWindows()
