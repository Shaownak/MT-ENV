import cv2
import serial
from ublox_gps import UbloxGps
import rospy
from std_msgs.msg import String

# To capture video from webcam.
cap = cv2.VideoCapture(0)

rospy.init_node("autonomous_arrow")

com_port_rover = ""
com_port_imu = ""
gps_com_port = ""
baudrate = 9600

rover_arduino = rospy.Publisher("/rover_controls", String, queue_size = 10)
gps_pub = rospy.Publisher("/gps_data", String, queue_size = 10)

imu = serial.Serial(com_port_imu, baudrate, timeout=1)
gps_port = serial.Serial(gps_com_port, baudrate=38400, timeout=1)

gps = UbloxGps(gps_port)

#Print Initial Frame size/property
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

cap.set(3, 1280)
cap.set(4, 720)

#Print Frame Size after capping
print(cap.get(3))
print(cap.get(4))

def parse_angle_from_IMUdata(data):
    return data

def rotate(degrees, direction):
    state = True
    current_angle = parse_angle_from_IMUdata(imu.readline())
    while state:
        new_current_angle = parse_angle_from_IMUdata(imu.readline())
        if new_current_angle > current_angle + degrees - 2 and new_current_angle < current_angle + degrees + 2:
            rover_arduino.publish("-")
            state = False
        else:
            #rover_arduino.write(bytes(str(degrees) + "|" + direction, "utf-8"))
            rover_arduino.write(bytes(direction, "utf-8"))

def get_gps_coords():
    coords = gps.geo_coords()
    return coords.lon, coords.lat

def find_arrow(arrow_state = False):
    arrow_found = False
    if arrow_state == True:
        distance = get_arrow_distance()
        x, y = get_arrow_coords()
    else:
        while not arrow_state:
            #processing to find arrow
            #if arrow found, set arrow_state to True
            if arrow_found == True:
                #get distance from realsense
                distance = get_arrow_distance()
                if distance < 2:
                    rover_arduino.publish("a")
                else:
                    rover_arduino.publish("-")
                    x, y = get_arrow_coords()
                    arrow_state = True
                    break
            else:
                rover_arduino.publish("a")

    return distance, x, y, arrow_state 

def get_arrow_coords():
    x = 0
    y = 0
    return x, y #this i:s the arrow x, y position. Midpoint position.

def get_arrow_distance():
    distance = 0
    return distance #this will be the distance of the arrow from the rover. It will be found from the realsense

def adjust_orientation(x, y):
     #Conditions
    if x <= 215 and x >= 0:
        print("Rotate Left 15 Degree")
        return 15, "a"
    elif x<=430 and x> 215:
        print("Rotate Left 5 Degree")
        return 5, "a"
    elif x >= 1065 and x <= 1280:   #Replace x with right top value Y
        print("Rotate Right 15 Degree")
        return 15, "d"
    elif x < 1065 and x >= 850:     #Replace x with right top value Y
        print("Rotate Right 5 Degree")
        return 5, "d"


def go_straight():
    rover_arduino.publish("w")

#look for arrow
#find arrow
#find arrow mid point
#go towards arrow until distance < 1
#going towards arrow --> adjust rover orientation depending where the midpoint of the arrow is 
#after distance < 1, get current gps coordinations
#look for arrow, restart cycle.


while True:
    gps_coords_state = False
    # Read the frame
    _, img = cap.read()

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect the faces
    #faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    # Draw the rectangle around each face
    #for (x, y, w, h) in faces:
        #cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #Draw Lines for screen separation
    cv2.line(img, (215, 0), (215, 720), (255, 0, 0), 1)
    cv2.line(img, (430, 0), (430, 720), (255, 210, 0), 1)
    cv2.line(img, (1065, 0), (1065, 720), (255, 0, 0), 1)
    cv2.line(img, (850, 0), (850, 720), (255, 210, 0), 1)

##########################################################################################################

    #DETECT OBJECT HERE AND BRING THE CENTER Upper left point of the object as X and the upper right point as Y
    #Initially Put Custom X & Y for tesing
    distance, x, y, arrow_state = find_arrow()
    while not gps_coords_state:
        distance, x, y, arrow_state = find_arrow(arrow_state = True)

        if distance > 2:
            degrees, direction = adjust_orientation(x, y)
            rotate(degrees, direction)
            go_straight()
        else:
            lon, lat = get_gps_coords()
            coord_string = str(lon) + " " + str(lat)
            gps_pub.publish(coord_string)
            gps_coords_state = True

    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

# Release the VideoCapture object
cap.release()
