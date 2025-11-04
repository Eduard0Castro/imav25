import cv2

import numpy as np

from enum import Enum

import rclpy
import yasmin
from yasmin_ros.yasmin_node import YasminNode

from yasmin import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros import MavDrone

from outdoor.find_a_person.constants import CENTRE_FRAME

class Manequim(Enum):
    
    OS_BP = 2,
    OS_WP = 1,
    YB_C = 3
    YY_C = 4


class MoveToPerson(State):

    """
    A state to centralize the drone with the target person
    """

    def __init__(self) -> None:

        super().__init__(outcomes={SUCCEED, ABORT})

        self.node = YasminNode.get_instance()

        yasmin.YASMIN_LOG_INFO("Move to person state has been initialized")
  

    def execute(self, blackboard: Blackboard)-> str:

        self.oakd              = blackboard['oakd_object']
        self.detection_nnqueue = blackboard['detection_nnqueue']
        self.labels            = blackboard['labels']
        self.preview_queue     = blackboard['preview_queue']
        self.drone : MavDrone  = blackboard['drone']

        self.manequim = Manequim.OS_WP.value

        self.stop = False
        self.keep_searching = True
        self.people = 0

        self.__coordinates_timer = self.node.create_timer(0.0001, self.get_coordinates)
        lat, long = self.drone.get_gps.latitude, self.drone.get_gps.longitude
        altitude = self.drone.get_gps.altitude - 17.0

        self.drone.offboard_position_gps_coords(lat, long, altitude, strategy="mavros")


        try:
            while not self.stop: 
                rclpy.spin_once(self.node, timeout_sec = 0.01)
                if self.keep_searching:
                    self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.1)


        except Exception as ex: 
            cv2.destroyAllWindows()
            if self.oakd.device: self.oakd.close()
            yasmin.YASMIN_LOG_ERROR(f"Move to person gets an error: {ex}")

            return ABORT
                
        else: return SUCCEED


    def go_to_person(self, detection) -> None:

        distance = detection.spatialCoordinates.z/1000.0

        move = distance - 1.5
        velocity_x = 3.0

        self.drone.offboard_velocity_timer(velocity_x, time = move/velocity_x)
        self.drone.land()            
        self.stop = True
        

    def get_coordinates(self) -> None:

        frame = self.oakd.getFrame(queue=self.preview_queue)
        inDet = self.detection_nnqueue.get()
        self.people_centre = list()

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        
        for detection in detections:

            try:
                label = self.labels[detection.label]
            except:
                label = detection.label
                
            if label == 'person':
                
                self.people_centre.append(detection.spatialCoordinates.z)

        if len(self.people_centre) > 0:
            self.people += 1
            if self.people == self.manequim:
                closest_person = np.argmin(self.people_centre)
                detection = detections[closest_person]
                self.keep_searching = False
                self.oakd.close()
                self.__coordinates_timer.destroy()
                self.go_to_person()
            
        else: yasmin.YASMIN_LOG_INFO("NO PERSON")
