import numpy as np
import cv2 as cv
import time
from picamera2 import Picamera2
from pymavlink import mavutil
import busio
import adafruit_vl53l0x

BOX_SIZE = 10 #50
THRESH = 0.01



class VehicleState(object):
	preFlight = "PREFLIGHT"
	takeoff = "TAKEOFF"
	auto = "AUTO"
	manual = "MANUAL"
	landing = "LANDING"
	landed = "LANDED"


def score(img, x, y):
    return np.sum(img[x-BOX_SIZE:x+BOX_SIZE, y-BOX_SIZE:y+BOX_SIZE])

#img = cv.GaussianBlur(img,(5,5),0)
def threshold(img):
    hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    gray = cv.inRange(hsv, (100, 0, 0), (255, 255, 255))
    return gray

def findStoppingPoint(gray):

    #gray = cv.GaussianBlur(gray,(7,7),0)
    #cv.namedWindow('dst2', cv.WINDOW_NORMAL)
    #cv.imshow('dst2',gray)
    #gray = np.uint8(gray)
    grayf = np.float32( gray)
    #grayf = cv.dilate(grayf,None)
    #grayf = cv.GaussianBlur(grayf,(10,10),0)

    dst = cv.cornerHarris(grayf,2,3,0.01)
    #result is dilated for marking the corners, not important
    #dst = cv.dilate(dst,None)
    # Threshold for an optimal value, it may vary depending on the image.


    # Create a mask to identify corners
    mask = np.zeros_like(grayf)
    
    # All pixels above a certain threshold are converted to white         
    mask[dst>THRESH*dst.max()] = 255

    # Convert corners from white to red.
    #img[dst > 0.01 * dst.max()] = [0, 0, 255]
    
    # Create an array that lists all the pixels that are corners
    coordinates = np.argwhere(mask)
    
    # Convert array of arrays to lists of lists
    coordinates_tuples = [tuple(l.tolist()) for l in list(coordinates)]
    print(f"no. of coordinates {len(coordinates_tuples)}")
    # Convert list to tuples
    #coordinates_tuples = [tuple(l) for l in coordinates_list]

    #filter
    filtered = [(x, y) for x, y in coordinates_tuples if 0.8*255*(2*BOX_SIZE)**2 > score(gray, x, y) > 0.7*255*(2*BOX_SIZE)**2]
    print(f"no. filtered {len(filtered)}")
    clusters = []
    CLUSTER_RADIUS_SQ = 10**2
    for x1, y1 in filtered:
        for i in range(0, len(clusters)):
            c, x2, y2 = clusters[i]
            if((x1-x2)**2+(y1-y2)**2 <= CLUSTER_RADIUS_SQ):
                x2 = (x1/(c+1)) + (x2*c/(c+1))
                y2 = (y1/(c+1)) + (y2*c/(c+1))
                c = c + 1
                clusters[i] = (c, x2, y2)
                break
        else:
            clusters.append((1, x1, y1))
    print(f"no. of clusters {len(clusters)}")
    if(len(clusters) < 4):
        return (None, None)
    clusters.sort(reverse = True)

    #dst = cv.dilate(dst,None)
    #dst = cv.dilate(dst,None)
    #img[dst>THRESH*dst.max()]=[0,255,255]

    #for c, x, y in clusters:
    #    img[int(x),int(y)] = [0,0,255];
    #    cv.circle(img, (int(y),int(x)), 25, (0,0,255), 3)

    
        
    centroid_x = int(np.mean([x for c, x, y in clusters[0:4]]))
    centroid_y = int(np.mean([y for c, x, y in clusters[0:4]]))
    #img[centroid_x-10:centroid_x+10,centroid_y-10:centroid_y+10] = [0,255,0];

    #cv.namedWindow('dst', cv.WINDOW_NORMAL)
    #cv.imshow('dst',img)


    #if cv.waitKey(0) & 0xff == 27:
    #   cv.destroyAllWindows()

    return (centroid_x, centroid_y)
    
### Threshold  image then extract white areas (assuming path is white) ###
def extract_white_areas(frame):
    ### Convert to grayscale ###
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    ### Threshold for white areas ###
    _, thresh = cv.threshold(gray, 200, 255, cv.THRESH_BINARY)
    return thresh

def getCentre(frame):
    frame_height, frame_width, _ = frame.shape
    frame_center = frame_width // 2
    return frame_center

### Process frame and detect white path ###
def process_frame(thresh, frame_center, rows, cols):
    ### Extract white area ###
    #thresh = extract_white_areas(frame)

    ### Find contours in the image ###
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if contours:
        ### Find largest contour ###
        largest_contour = max(contours, key=cv.contourArea)
        M = cv.moments(largest_contour)
        
        if M["m00"] > 0:  ### Ensure there is a real contour ###
            ### Calculate the centroid of the largest contour ###
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            ### Draw the contour and centroid on the frame ###
            #cv.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
            #cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            ### Calculate path best fit line ###
            #rows, cols = frame.shape[:2]
            [vx, vy, x, y] = cv.fitLine(largest_contour, cv.DIST_L2, 0, 0.01, 0.01)
            ### Use two points to represent the line ###
            lefty = int((-x * vy / vx) + y)
            righty = int(((cols - x) * vy / vx) + y)
            
            ### Draw line ###
            #cv.line(frame, (cols - 1, righty), (0, lefty), (255, 0, 0), 2)

            ### Calculate path angle ###
            angle = np.arctan2((righty - lefty), (cols - 1)) #* 180 / np.pi
            if angle < 0:
                angle += np.pi
            

            ### Determine path direction ###
            if cx < frame_center - 50:
                direction = 'left'
            elif cx > frame_center + 50:
                direction = 'right'
            else:
                direction = 'straight'

            ### Display in window angle and direction ###
            #cv.putText(frame, f"Direction: {direction}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            #cv.putText(frame, f"Angle: {angle:.2f}", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            
            return direction, angle, cx, cy, lefty, righty
    return None, None, None, None, None, None

def normalDeviation(angle, cx, cy, height=480, width=640):
    (nx, ny) = (np.sin(angle), -np.cos(angle))
    (px, py) = (cx-(width/2), cy-(height/2))
    return nx*px + ny*py

DEAD_ZONE = 50

def controlAction(angle, deviation):
    vt = 0.5
    vn = 0
    if deviation > DEAD_ZONE:
        vn = 0.2
    elif deviation < -DEAD_ZONE:
        vn = -0.2
    return (vt*np.cos(angle) + vn*np.sin(angle), vt*np.sin(angle) - vn*np.cos(angle))

def set_guided(mav_connection):
     # Change mode to guided (Ardupilot) or takeoff (PX4)
    mode_id = mav_connection.mode_mapping()["GUIDED"]
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

def set_auto(mav_connection):
     # Change mode to guided (Ardupilot) or takeoff (PX4)
    mode_id = mav_connection.mode_mapping()["AUTO"]
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

def arm(mav_connection):
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

def disarm(mav_connection):
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

def land(mav_connection):
    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mode_id = mav_connection.mode_mapping()["LAND"]
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode (LAND) ACK:  {ack_msg}")

def set_velocity_body(mav_connection, vx, vy, vz, yaw=0):
    mav_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b100111000111),
                        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0))
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"set_velocity_body ACK:  {ack_msg}")

def takeoff(mav_connection, targetHeight, wait_ready = True):
    state = VehicleState.takeoff
    msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    startHeightGlobal = msg.alt / 1000
    print(f"startHeightGlobal: {startHeightGlobal}")
    print(f"GLOBAL_POSITION_INT: {msg}")
    targetHeightGlobal = startHeightGlobal + targetHeight
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, targetHeight)
		
		
    print("Vehicle is taking off!")
    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    while wait_ready:
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        currentAlt = msg.alt / 1000
        print(f"GLOBAL_POSITION_INT: {msg}")

        if state != VehicleState.takeoff:
            print("Err: Takeoff terminated unexpectedly with state", state)
            return False
        print(" Current relativealtitude: ", currentAlt - startHeightGlobal, "\n"); 
			#Break and return from function just below target altitude.        
        if currentAlt - startHeightGlobal >= targetHeight * TAKEOFF_ALT_SCALER: 
            print("Reached target altitude")
            state = VehicleState.auto
            break

def isObstacles(vl53):
    distance = vl53.range
    stop = False
    if 1000 > distance > 100: #mm
        stop = True
    print("Distance: {}mm, stop = {}".format(distance, stop))
    return stop

print("Initialize I2C bus and VL53L0X")
i2c = busio.I2C(3, 2)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

print("initilisaing camera")
cam = Picamera2()
height = 480
width = 640
middle = (int(width / 2), int(height / 2))
cam.configure(cam.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))

cam.start()
print("camera started")

mav_connection = mavutil.mavlink_connection('tcp:localhost:14540')
print("waiting for MAVLINK heartbeat")
mav_connection.wait_heartbeat()

print("main: set_guided")
set_guided(mav_connection)
print("main: arm")
arm(mav_connection)

print("taking off!")
takeoff(mav_connection, 0.5, wait_ready=False)
time.sleep(3)


#filename = 'StopMark.jpg'
#img = cv.imread(filename)
#video = cv.VideoCapture(0)
#print(f"got video {video.isOpened()}")
frame_width = width
frame_height = height

# Define the codec and create VideoWriter object
#fourcc = cv.VideoWriter_fourcc(*'mp4v')
#out = cv.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))
#print("starting...")

FLIGHT_DURATION = 30
timeout = time.time() + FLIGHT_DURATION
stoptime = None
isStopped = False
STOPPING_TIME = 5
ESCAPE_TIME = 5
while True:
    img = cam.capture_array()

    thresh = threshold(img)
    if(stoptime == None):
        (x, y) = findStoppingPoint(thresh)
        if(x != None):
            stoptime = time.time()
            isStopped = True
    else:
        if(time.time() > stoptime+STOPPING_TIME):
            isStopped = False
        if(time.time() > stoptime+STOPPING_TIME+ESCAPE_TIME):
            stoptime = None
    
    rows, cols = img.shape[:2]
    direction, angle, cx, cy, lefty, righty = process_frame(thresh, getCentre(img), rows, cols)
    
    if((cx != None) and not isObstacles(vl53) and not isStopped):
        d = normalDeviation(angle, cx, cy)
        (vx, vy) = controlAction(angle, d)
        set_velocity_body(mav_connection, vx, vy, 0)
    else:
        set_velocity_body(mav_connection, 0, 0, 0)
    if time.time() > timeout:
        break

set_velocity_body(mav_connection, 0, 0, 0)
print("landing!")
land(mav_connection)
time.sleep(3)
disarm(mav_connection)
print("done!")
#time.sleep(1)
#if cv.waitKey(0) & 0xff == 27:
#    cv.destroyAllWindows()