LABELS = ["background", "aeroplane", "bicycle", "bird", "boat", 
          "bottle", "bus", "car", "cat", "chair", "cow",
          "diningtable", "dog", "horse", "motorbike", "person",
          "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
PERSON = "person"
SHAPE = (300, 300)

INIT_MISSION_ATTEMPTS = 3
OAKD_INIT_ATTEMPTS = 3

HEADING_TOLERANCE = 30

TAKE_OFF_ALTITUDE = 20.0
TARGET_COORDINATE = (18.9438158, -98.188531)
DETECTION_ALTITUDE = 3.0

PERSON_LAND_DISTANCE = 1.5
DISTANCE_TOLERANCE = 0.25

PRE_CENTERING_TIMEOUT = 30.0
MOVE_TO_PERSON_TIMEOUT = 240.0

PERSON_CENTER_KP_Y = 0.00031
PERSON_CENTER_KP_Z = 0.00031
GOTO_PERSON_KP_X = 0.001
MAX_VELOCITY_YZ = 0.3
MAX_VELOCITY_X = 0.5
MIN_VELOCITY_X = 0.05

PRE_CENTER_TOLERANCE = 30

#Custom outcomes:
CAMERA_CRASH = "CAMERA_CHRASH"
RETURN_SEARCH = "RETURN_SEARCH"
RETURN_MOVE = "RETURN_MOVE"

