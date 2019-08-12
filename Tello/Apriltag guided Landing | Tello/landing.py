# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

import apriltag
from djitellopy import Tello
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time
import imutils as im

# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 25


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
    """

    def __init__(self):
        # Init pygame
        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()
        self.telloPose = np.zeros((1,3))
        self.telloEulerAngles = np.zeros((1,3))

        self.rcOut=np.zeros(4)
        

        self.poseQueue = np.zeros((7,3))
        self.supreQueue = np.zeros((7,3))

        self.flag = 0
        self.telloPoseVariance = np.zeros(3)
        self.telloPoseMean = np.zeros(3)
        self.tello.TIME_BTW_RC_CONTROL_COMMANDS = 20

        self.R = np.zeros((3,3))

        # self.telloPose = np.array([])
            # self.telloEulerAngles = EulerAngles

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        should_stop = False

        Height = 100
        while not should_stop:
            if frame_read.stopped:
                frame_read.stop()
                break

            frameBGR = np.copy(frame_read.frame)
            # frameBGR = np.rot90(frameBGR)
            # frameBGR = np.flipud(frameBGR)
            frame2use = im.resize(frameBGR,width=720)
            

            key = cv2.waitKey(1) & 0xFF;

            
            K = np.array([[7.092159469231584126e+02,0.000000000000000000e+00,3.681653710406367850e+02],[0.000000000000000000e+00,7.102890453175559742e+02,2.497677007139825491e+02],[0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])
            dist = np.array([-1.428750372096417864e-01,-3.544750945429044758e-02,1.403740315118516459e-03,-2.734988255518019593e-02,1.149084393996809700e-01])
            K_inv = np.linalg.inv(K)

            h , w = frame2use.shape[:2]

            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K,dist,(w,h),1,(w,h))

            mapx,mapy = cv2.initUndistortRectifyMap(K,dist,None,newcameramtx,(w,h),5)
            dst = cv2.remap(frame2use,mapx,mapy,cv2.INTER_LINEAR)

            x,y,w,h = roi
            dst = dst[y:y+h,x:x+w]
            # print("ROI: ",x,y,w,h)

            cv2.imshow("Orignal",frame2use)
            cv2.imshow("rectified",dst)
            # gray = cv2.cvtColor(frame2use, cv2.COLOR_BGR2GRAY)
            gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
            detector = apriltag.Detector()
            result = detector.detect(gray)

            if len(result) != 0:
                self.flag = 1
            else :
                self.flag = 0

            self.FindPose(result,K_inv)

            if self.flag == 1:
                varN = np.linalg.norm(self.telloPoseVariance)
                print "varN",varN
                Pose = self.telloPoseMean


                xEr = 0 - Pose[0]   
                yEr = 0 - Pose[1]
                zEr = Height - Pose[2]
                ErrorN = np.linalg.norm([xEr,yEr,zEr])

                print "Height",Height

                print "ErrorN",ErrorN

                if varN < 370 and key == ord("e"):
                    if varN < 120 and ErrorN < 10:
                        Height = Height -5
                        print "#######################################################################"

                    if Height <30:
                        self.tello.land()
                    kp = 3

                    MtnCmd = np.matmul(1,[kp*xEr,kp*yEr,kp*zEr])
                    self.rcOut = [MtnCmd[0],MtnCmd[1],MtnCmd[2],0]
                    

                    if self.rcOut[0] > 60:
                        self.rcOut[0] = 60
                    elif self.rcOut[0] < -60:
                        self.rcOut[0] = -60

                    if self.rcOut[1] > 60:
                        self.rcOut[1] = 60
                    elif self.rcOut[1] < -60:
                        self.rcOut[1] = -60

                    if self.rcOut[2] > 60:
                        self.rcOut[2] = 60
                    elif self.rcOut[2] < -60:
                        self.rcOut[2] = -60

                    # elif self.rcOut[1] > 60:
                        # self.rcOut[1] = 60
                    # elif self.rcOut[1] < -60:
                        # self.rcOut[1] = -60
                else :
                    if key == ord("w"):
                        self.rcOut[1] = 50
                    elif key == ord("a"):
                        self.rcOut[0] = -50
                    elif key == ord("s"):
                        self.rcOut[1] = -50
                    elif key == ord("d"):
                        self.rcOut[0] = 50
                    elif key == ord("u"):
                        self.rcOut[2] = 50
                    elif key == ord("j"):
                        self.rcOut[2] = -50
                    else:
                        self.rcOut = [0,0,0,0]



            else:
                if key == ord("w"):
                    self.rcOut[1] = 50
                elif key == ord("a"):
                    self.rcOut[0] = -50
                elif key == ord("s"):
                    self.rcOut[1] = -50
                elif key == ord("d"):
                    self.rcOut[0] = 50
                elif key == ord("u"):
                    self.rcOut[2] = 50
                elif key == ord("j"):
                    self.rcOut[2] = -50
                else:
                    self.rcOut = [0,0,0,0]

            print self.rcOut
            self.tello.send_rc_control(int(self.rcOut[0]),int(self.rcOut[1]),int(self.rcOut[2]),int(self.rcOut[3]))
            self.rcOut = [0,0,0,0]

            if key == ord("q"):
                break
            if key == ord("t"):
                self.tello.takeoff()    
            if key == ord("l"):
                self.tello.land()
                Height = 100

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(1 / FPS)

        # Call it always before finishing. I deallocate resources.
        self.tello.end()

    def FindPose(self,result,K_inv):
        if  self.flag == 1:
            center = result[0].center
            H = np.array(result[0].homography)
            # print H
            h1h2h3 = np.matmul(K_inv,H)

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
            T = T*100/17.5
            r = np.matmul(u,M)
            R = np.matmul(r,vh) # Rotation matrix
            T = T

            self.R = R

            T_t = np.reshape(T,(3,1))
            neg_Rt_T = -1*np.dot(R.T,T_t)
            f = np.array([[0,0,0,1]])

            
            if neg_Rt_T[2,0] < 0:
                flag = -1
            else:
                flag = 1

            neg_Rt_T[2,0] = neg_Rt_T[2,0]*flag
            neg_Rt_T[0,0] = neg_Rt_T[0,0]*(-1)
            Pose = neg_Rt_T.T
            EulerAngles = rotationMatrixToEulerAngles(R.T)
            self.telloPose = Pose
            self.telloEulerAngles = EulerAngles

            self.poseQueue = np.roll(self.poseQueue,1,axis = 0)
            self.poseQueue[0,:] = [Pose[0,0],Pose[0,1],Pose[0,2]]

            self.telloPoseVariance = np.var(self.poseQueue,axis=0)
            self.telloPoseMean = np.mean(self.poseQueue,axis = 0)

            # print "PoseQueue",self.poseQueue
            print "PoseMean",self.telloPoseMean
            # print"telloPoseVariance",self.telloPoseVariance
            # Pose,EulerAngles
            return


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()


#________________________________________________________________________________________________________________________________________________-#
# # Checks if a matrix is a valid rotation matrix.
# def isRotationMatrix(R) :
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype = R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6
 
 
# # Calculates rotation matrix to euler angles
# # The result is the same as MATLAB except the order
# # of the euler angles ( x and z are swapped ).
# def rotationMatrixToEulerAngles(R) :
 
#     assert(isRotationMatrix(R))
     
#     sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
#     singular = sy < 1e-6
 
#     if  not singular :
#         x = np.arctan2(R[2,1] , R[2,2])
#         y = np.arctan2(-R[2,0], sy)
#         z = np.arctan2(R[1,0], R[0,0])
#     else :
#         x = np.arctan2(-R[1,2], R[1,1])
#         y = np.arctan2(-R[2,0], sy)
#         z = 0
#     return np.array([x, y, z])

# import apriltag
# from djitellopy import Tello
# import cv2
# import pygame
# from pygame.locals import *
# import numpy as np
# import time
# import imutils as im

# # Speed of the drone
# S = 60
# # Frames per second of the pygame window display
# FPS = 25


# class FrontEnd(object):
#     """ Maintains the Tello display and moves it through the keyboard keys.
#         Press escape key to quit.
#         The controls are:
#             - T: Takeoff
#             - L: Land
#             - Arrow keys: Forward, backward, left and right.
#             - A and D: Counter clockwise and clockwise rotations
#             - W and S: Up and down.
#     """

#     def __init__(self):
#         # Init pygame
#         # Init Tello object that interacts with the Tello drone
#         self.tello = Tello()

#     def run(self):

#         if not self.tello.connect():
#             print("Tello not connected")
#             return

#         # In case streaming is on. This happens when we quit this program without the escape key.
#         if not self.tello.streamoff():
#             print("Could not stop video stream")
#             return

#         if not self.tello.streamon():
#             print("Could not start video stream")
#             return

#         frame_read = self.tello.get_frame_read()

#         should_stop = False

#         name = "outImg"
#         i = 0

#         while not should_stop:
#             if frame_read.stopped:
#                 frame_read.stop()
#                 break

#             frameBGR = np.copy(frame_read.frame)
#             # frameBGR = np.rot90(frameBGR)
#             frameBGR = np.flipud(frameBGR)
#             frame2use = im.resize(frameBGR,width=720)
            

#             key = cv2.waitKey(1) & 0xFF ;

            
#             K = np.array([[7.908929771326191940e+02,0.000000000000000000e+00,1.967509686027607643e+02],[0.000000000000000000e+00,7.674452703898529080e+02,5.028845988338624124e+02],[0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])
#             dist = np.array([-1.428750372096417864e-01,-3.544750945429044758e-02,1.403740315118516459e-03,-2.734988255518019593e-02,1.149084393996809700e-01])
#             K_inv = np.linalg.inv(K)

#             # h , w = frame2use.shape[:2]

#             # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K,dist,(w,h),1,(w,h))

#             # mapx,mapy = cv2.initUndistortRectifyMap(K,dist,None,newcameramtx,(w,h),5)
#             # dst = cv2.remap(frame2use,mapx,mapy,cv2.INTER_LINEAR)

#             # x,y,w,h = roi
#             # dst = dst[y:y+h,x:x+w]
#             # print("ROI: ",x,y,w,h)

#             cv2.imshow("lkgs",frame2use)
#             # cv2.imshow("rectified",dst)
#             gray = cv2.cvtColor(frame2use, cv2.COLOR_BGR2GRAY)
#             # gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
#             detector = apriltag.Detector()
#             result = detector.detect(gray)
            

#             if len(result) != 0:
#                 center = result[0].center
#                 H = np.array(result[0].homography)
#                 # print H
#                 h1h2h3 = np.dot(K_inv,H)

#                 h1T = h1h2h3[:,0]
#                 h2T = h1h2h3[:,1]
#                 h3T = h1h2h3[:,2]
                
#                 # h1Xh2T = np.array([H[1,0]*H[2,1] - H[2,0]*H[1,1], H[0,1]*H[2,0] - H[0,0]*H[2,1] , H[0,0]*H[1,1] - H[0,1]*H[1,0]])
#                 h1Xh2T = np.cross(h1T,h2T)

#                 print "h1Xh2T",h1Xh2T
#                 # print "i",i       

#                 # print h1T
#                 # print h2T
#                 # print h1Xh2T

#                 h1_h2_h1Xh2T = np.array([h1T,h2T,h1Xh2T])
#                 h1_h2_h1Xh2 = np.transpose(h1_h2_h1Xh2T)

#                 u, s, vh = np.linalg.svd(h1_h2_h1Xh2, full_matrices=True)

#                 uvh = np.dot(u,vh)
#                 det_OF_uvh = np.linalg.det(uvh)

#                 M = np.array([[1,0,0],[0,1,0],[0,0,det_OF_uvh]])

#                 r = np.dot(u,M)
#                 R = np.dot(r,vh)

#                 # print R
#                 # if roll <0:
#                     # roll = 2*np.pi + roll

#                 # if pitch <0:
#                 #   pitch = 2*np.pi + pitch

#                 # if yaw <0:
#                 #   yaw = 2*np.pi + yaw 

#                 T = h3T/np.linalg.norm(h1T)

#                 if T[2] < 0:
#                     flag = -1
#                 else : 
#                     flag = 1

#                 mat = rotationMatrixToEulerAngles(R*flag)

#                 roll = mat[0]
#                 pitch =  mat[1]
#                 yaw = mat[2]

#                 print "Translation",T*flag
#                 print "roll",roll,"pitch",pitch,"yaw",yaw

    
#             if key == ord("c"):
#                 new = name + str(i) + ".jpg"
#                 print new
#                 i = i+1
#                 cv2.imwrite(new,frame)

#             key = cv2.waitKey(1) & 0xFF ;
#             if key == ord("q"):
#                 break
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#             frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)

#             frame = np.rot90(frame)
#             frame = np.flipud(frame)
#             # a = frameGray.dtype
#             # print a
            
#             time.sleep(1 / FPS)

#         # Call it always before finishing. I deallocate resources.
#         self.tello.end()


# def main():
#     frontend = FrontEnd()

#     # run frontend
#     frontend.run()


# if __name__ == '__main__':
#     main()




#     