import numpy as np

import time

import yasmin
from yasmin_ros.yasmin_node import YasminNode

from yasmin import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros import MavDrone
from mirela_sdk.image_processing.camera import OakdCam
from mirela_sdk.control.pid import PIDController

from outdoor.find_a_person.constants import (
    SHAPE, 
    PERSON, 
    PRE_CENTERING_TIMEOUT,
    MOVE_TO_PERSON_TIMEOUT,
    PERSON_CENTER_KP_Y,
    PERSON_CENTER_KP_Z,
    MAX_VELOCITY_YZ,
    MAX_VELOCITY_X,
    MIN_VELOCITY_X,
    GOTO_PERSON_KP_X,
    PERSON_LAND_DISTANCE,
    PRE_CENTER_TOLERANCE,
    DISTANCE_TOLERANCE,
    CAMERA_CRASH
    )


class MoveToPerson(State):

    """
    A state to centralize the drone with the target person
    """

    def __init__(self) -> None:

        super().__init__(outcomes={SUCCEED, 
                                   CAMERA_CRASH, 
                                   ABORT})

        self.pid_x = None
        self.pid_y = None
        self.pid_z = None
        self.pre_centered = False
        self.time_to_land = False
        self.timeout_controller = False

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard) -> None:

        self.drone: MavDrone   = blackboard[            'drone']
        self.oakd:  OakdCam    = blackboard[             'oakd']
        self.preview_queue     = blackboard[    'preview_queue']
        self.detection_nnqueue = blackboard['detection_nnqueue']
        self.frame_centre = (SHAPE[0]//2, SHAPE[1]//2)

        blackboard['camera_crash_state'] = "MOVE TO PERSON"

        if self.pid_y is None:
            self.pid_y = PIDController(
                                    kp = PERSON_CENTER_KP_Y,
                                    ki = 0.0,
                                    kd = 0.0,
                                    setpoint = self.frame_centre[0],
                                    output_limits = (-MAX_VELOCITY_YZ, MAX_VELOCITY_YZ),
                                    )
        if self.pid_z is None:
            self.pid_z = PIDController(
                                    kp = PERSON_CENTER_KP_Z,
                                    ki = 0.0,
                                    kd = 0.0,
                                    setpoint = self.frame_centre[1],
                                    output_limits = (-MAX_VELOCITY_YZ, MAX_VELOCITY_YZ),
                                    )
            
        if self.pid_x is None:
            self.pid_x = PIDController(
                                    kp = GOTO_PERSON_KP_X,
                                    ki = 0.0,
                                    kd = 0.0,
                                    setpoint = PERSON_LAND_DISTANCE,
                                    output_limits = (MIN_VELOCITY_X, MAX_VELOCITY_X),
                                    )
            

        try: self.__detection_loop()
        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"Move to person state gets an error: {ex}")
            if self.oakd.device.isClosed(): return CAMERA_CRASH
            return ABORT
        else:
            if self.timeout_controller: 
                yasmin.YASMIN_LOG_ERROR("Move to person timeout: ABORTING")
                return ABORT
            return SUCCEED

    def __detection_loop(self) -> None:

        self.__pre_centering_start  = time.time()
        self.__move_to_person_start = time.time()

        while not self.time_to_land and not self.timeout_controller:

            inDet = self.detection_nnqueue.get()

            people_detection = list()
            people_distance = list()

            detections = inDet.detections

            for detection in detections:
                
                try:
                    label = self.labels[detection.label]
                except:
                    label = detection.label
                
                if label == PERSON:
                    people_detection.append(detection)
                    people_distance.append(detection.spatialCoordinates.z)
            
            if len(people_detection) > 0:
                closest_person = np.argmin(people_distance)
                self.controller(people_detection[closest_person])


    def controller(self, detection) -> None:


        center_person_x = (detection.xmax*SHAPE[0] + detection.xmin*SHAPE[0])//2
        center_person_Y = (detection.ymax*SHAPE[1] + detection.ymin*SHAPE[1])//2
        distance_from_person = detection.spatialCoordinates.z/1000 # mm to m
        
        vel_y = self.pid_y.update(center_person_x)
        vel_z = self.pid_z.update(center_person_Y)
        vel_x = self.pid_x.update(distance_from_person)

        if not self.pre_centered:
            if time.time() - self.__pre_centering_start > PRE_CENTERING_TIMEOUT:
                self.timeout_controller = True
                return

            yasmin.YASMIN_LOG_INFO(f"Pre-centering:\nvel_y = {vel_y}\nvel_z = {vel_z}")

            self.drone.offboard_velocity(0.0, vel_y, vel_z, 0.0, False)

            if(abs(SHAPE[0] - center_person_x) <= PRE_CENTER_TOLERANCE and
            abs(SHAPE[1] - center_person_Y) <= PRE_CENTER_TOLERANCE):
                self.pre_centered = True
        else:
            if time.time() - self.__move_to_person_start > MOVE_TO_PERSON_TIMEOUT:
                self.timeout_controller = True
                return

            yasmin.YASMIN_LOG_INFO(f"Move to person:\nvel_x = {vel_x}\n \
                                   vel_y = {vel_y}\nvel_z = {vel_z}")

            self.drone.offboard_velocity(vel_x, vel_y, vel_z, 0.0, False)

            if(abs(SHAPE[0] - center_person_x) <= PRE_CENTER_TOLERANCE and
            abs(SHAPE[1] - center_person_Y) <= PRE_CENTER_TOLERANCE and
            abs(PERSON_LAND_DISTANCE - distance_from_person) <= DISTANCE_TOLERANCE):
                
                self.time_to_land = True
                return