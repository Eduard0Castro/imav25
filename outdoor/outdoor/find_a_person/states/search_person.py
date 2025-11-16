import numpy as np

import rclpy
import yasmin
from yasmin_ros.yasmin_node import YasminNode

from yasmin import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros import MavDrone
from mirela_sdk.image_processing.camera import OakdCam

from outdoor.find_a_person.constants import (SHAPE, 
                                             PERSON, 
                                             HEADING_TOLERANCE, 
                                             CAMERA_CRASH)


class SearchPerson(State):

    """
    A state to search for person after reaching the coordinate
    """

    def __init__(self) -> None:

        super().__init__(outcomes={SUCCEED, CAMERA_CRASH, ABORT})

        self.node = YasminNode.get_instance()

        self.people_images      = list()
        self.width, self.height = SHAPE
        self.current_heading    = None
        self.target_person_detection = None
  

    def execute(self, blackboard: Blackboard)-> str:

        self.oakd : OakdCam    = blackboard['oakd_object']
        self.detection_nnqueue = blackboard['detection_nnqueue']
        self.labels            = blackboard['labels']
        self.preview_queue     = blackboard['preview_queue']
        self.drone : MavDrone  = blackboard['drone']

        blackboard['camera_crash_state'] = "SEARCH PERSON"

        self.stop = False
        self.keep_searching = True

        self.__detection_timer = self.node.create_timer(0.0001, self.get_detection)

        try:
            while not self.stop: 
                rclpy.spin_once(self.node, timeout_sec = 0.01)
                if self.keep_searching:
                    self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.3)
                else: self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.0)

            blackboard['target_person_detection'] = self.target_person_detection

        except Exception as ex: 

            yasmin.YASMIN_LOG_ERROR(f"Move to person gets an error: {ex}")

            if self.oakd.device.isClosed(): return CAMERA_CRASH

            return ABORT
                
        else: return SUCCEED


    def cut_image(self, frame: np.array, detection) -> np.array:

        x1 = int(detection.xmin * self.width)
        x2 = int(detection.xmax * self.width)
        y1 = int(detection.ymin * self.height)
        y2 = int(detection.ymax * self.height)

        return frame[y1:y2, x1:x2]


    def detect_correct_person(self, image: np.array) -> bool:
        ...
       

    def get_detection(self) -> None:


        frame = self.oakd.getFrame(queue = self.preview_queue)
        inDet = self.detection_nnqueue.get()
        people_centre = list()
        people_detection = list()

        detections = inDet.detections
        heading = self.drone.get_heading
        
        for detection in detections:

            try:
                label = self.labels[detection.label]
            except:
                label = detection.label
                
            if label == PERSON:
                people_detection.append(detection)
                people_centre.append(detection.spatialCoordinates.z)

        if len(people_detection) > 0:

            if heading < self.current_heading - HEADING_TOLERANCE or \
               heading > self.current_heading + HEADING_TOLERANCE or \
               self.current_heading is None:

                self.keep_searching = False
                self.current_heading = heading

                closest_person = np.argmin(people_centre)
                detection = people_detection[closest_person]
                person_image = self.cut_image(frame=frame, detection=detection)
                has_found = self.detect_correct_person(person_image)

                if has_found: 
                    self.target_person_detection = detection
                    self.stop = True
                    self.__detection_timer.destroy()
                else: self.keep_searching = True
            
        else: yasmin.YASMIN_LOG_INFO("NO PERSON")