import cv2

from pathlib import Path

import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin.state import State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.image_processing import OakdCam
from mirela_sdk.control.mavros import MavDrone

from outdoor.mapping.constants import COORDINATES


class FullMapping(State):

    PATH = Path(__file__).resolve().absolute().parent.parent / 'mapping_photos'

    def __init__(self) -> None:

        super().__init__(outcomes = {SUCCEED, ABORT})

        self.coordinates = COORDINATES
        self.node = YasminNode.get_instance()
        self.photo = 1


    def execute(self, blackboard: Blackboard) -> None:
        
        self.oakd : OakdCam  = blackboard[ 'oakd']
        self.drone: MavDrone = blackboard['drone']

        try:
            for index, coordinate in enumerate(self.coordinates):
                if coordinate is not None:
                    lat, long = coordinate
                    heading = self.drone.gps_controller.calculate_bearing(lat, long)
                    self.drone.offboard_position_gps_coords(lat, long, 
                                                            heading=heading, 
                                                            strategy="mavros")
                    self.drone.delay(2.0)
                    frame = self.oakd.get_frame()

                    self.photo = "0" + str(self.photo) if self.photo < 10 else self.photo
                    cv2.imwrite(f"{FullMapping.PATH}/photo_{self.photo}.jpg", frame)
                    self.node.get_logger().info(f"Photo {self.photo}")
                    self.photo = int(self.photo) + 1

                    self.drone.delay(1)
                    self.coordinates[index] = None

        except Exception as ex:
            if self.oakd.device: self.oakd.close()
            yasmin.YASMIN_LOG_ERROR(f"Full mapping mission gets an error: {ex}")
            return ABORT
        else: return SUCCEED


        

