import cv2

import numpy as np

import rclpy
import yasmin
from yasmin_ros.yasmin_node import YasminNode

from yasmin import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros import MavDrone

from outdoor.find_a_person.constants import CENTRE_FRAME

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

        self.stop = False
        self.keep_searching = True

        self.__coordinates_timer = self.node.create_timer(0.0001, self.get_coordinates)

        try:
            while not self.stop: rclpy.spin_once(self.node, timeout_sec = 0.01)

        except Exception as ex: 
            cv2.destroyAllWindows()
            if self.oakd.device: self.oakd.close()
            yasmin.YASMIN_LOG_ERROR(f"Move to person gets an error: {ex}")

            return ABORT
                
        else: return SUCCEED

    def search_for_person(self)-> None:

        lat, long = self.drone.get_gps.latitude, self.drone.get_gps.longitude
        altitude = self.drone.get_gps.altitude - 25.0

        self.drone.offboard_position_gps_coords(lat, long, altitude, strategy="mavros")

        while self.keep_searching:
            rclpy.spin_once(self.node)
            self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.1)

    def go_to_person(self, detection) -> None:

        distance = detection.spatialCoordinates.z

        move = distance - 1.5
        velocity_x = 3.0

        self.drone.offboard_velocity_timer(velocity_x, time = move/velocity_x)
        self.drone.land()
        

    def get_coordinates(self) -> None:

        frame = self.oakd.getFrame(queue=self.preview_queue)
        inDet = self.detection_nnqueue.get()
        self.people_centre = list()

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        
        for detection in detections:

            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = self.labels[detection.label]
            except:
                label = detection.label

            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), 
                        cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), 
                        (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", 
                        (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", 
                        (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", 
                        (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)
                
            if label == 'person':
                
                self.people_centre.append(detection.spatialCoordinates.z)

        if len(self.people_centre) > 0:
            closest_person = np.argmin(self.people_centre)
            detection = detections[closest_person]
            self.keep_searching = False
            
        else: yasmin.YASMIN_LOG_INFO("NO PERSON")

        cv2.imshow("Teste", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            self.oakd.close()
            self.__coordinates_timer.destroy()
            self.stop = True
