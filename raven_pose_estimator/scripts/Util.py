import tfx
import image_geometry
import rospy
import cv

def convertStereo(self, u, v, disparity, info):
    """
    Converts two pixel coordinates u and v along with the disparity to give PointStamped       
    """
    stereoModel = image_geometry.StereoCameraModel()
    stereoModel.fromCameraInfo(info['l'], info['r'])
    (x,y,z) = stereoModel.projectPixelTo3d((u,v), disparity)

    cameraPoint = PointStamped()
    cameraPoint.header.frame_id = self.leftInfo.header.frame_id
    cameraPoint.header.stamp = rospy.Time.now()
    cameraPoint.point = Point(x,y,z)


def determineROI(info):
    width = 250
    height = 250
    pose = tfx.pose([0,0,0], [0,0,0,1], frame='/tool_L', stamp=rospy.Time.now())
    tf_tool_to_cam = tfx.lookupTransform('/left_optical_frame','/tool_L', wait=2)
    pose = tf_tool_to_cam * pose
    stereoModel = image_geometry.StereoCameraModel()
    stereoModel.fromCameraInfo(info['l'], info['r'])
    (u_l, v_l), (u_r, v_r) = stereoModel.project3dToPixel(pose.position.list)
    print "LEFT CENTER : "+str(u_l)+", "+str(v_l)
    print "RIGHT CENTER: "+str(u_r)+", "+str(v_r)
    return (u_l, v_l), (u_r, v_r)

def getROI(image, pixel, half_width=100, half_height=100):
    width = 2 * half_width
    height = 2 * half_height
    heightOffset = int(pixel[1]) - half_height
    widthOffset = int(pixel[0]) - half_width
    return image[heightOffset : heightOffset + height, widthOffset : widthOffset + width]

def threshold(image, hsvImg, threshImg, lowerHsv, upperHsv):
    cv.Smooth(image, image, cv.CV_GAUSSIAN, 3, 0)
    cv.CvtColor(image, hsvImg, cv.CV_BGR2HSV)
    cv.InRangeS(hsvImg, lowerHsv, upperHsv, threshImg)
    cv.Erode(threshImg, threshImg, None, 1)
    cv.Dilate(threshImg, threshImg, None, 1)
    return threshImg

def find_centroid(threshImg):
    mat = cv.GetMat(threshImg)
    yxCoords = []
    for x in range(mat.width):
        for y in range(mat.height):
            if mat[y,x] > 0.0:
                yxCoords.append((y,x))
    # check if any color is present
    if len(yxCoords) == 0:
        return (False,0,0,0,0)
    else:
        yCentroid = sum([y for y,x in yxCoords])/len(yxCoords)
        xCentroid = sum([x for y,x in yxCoords])/len(yxCoords)
        # find nearest color pixel to centroid (based on euclidean distance)
        distFromCentroid = [((y-yCentroid)**2 + (x-xCentroid)**2)**.5 for y,x in yxCoords]
        yClose, xClose = yxCoords[distFromCentroid.index(min(distFromCentroid))]
        return (True, xClose, yClose, xCentroid, yCentroid)

def showImage(image, name):
    cv.NamedWindow(name, cv.CV_WINDOW_AUTOSIZE)
    cv.ShowImage(name, image)
    cv.WaitKey(50)
