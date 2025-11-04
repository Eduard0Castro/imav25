import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin.state import State
from yasmin.blackboard import Blackboard

from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing import OakdCam

class InitializeMission(State):

    def __init__(self)-> None:

        super().__init__(outcomes={SUCCEED, ABORT})

        self.node = YasminNode.get_instance()
        self.drone = MavDrone(self.node, False)


    def execute(self, blackboard: Blackboard) -> str:
        
        blackboard["drone"] = self.drone
        self.drone.check_driver_node(2.0)
        lat =  18.9438158
        long = -98.188531

        try:
            self.drone.arm_takeoff(20.0)
            heading = self.drone.gps_controller.calculate_bearing(lat, long)
            self.drone.offboard_position_gps_coords(latitude = lat, 
                                                    longitude = long, 
                                                    heading= heading,
                                                    strategy = "mavros")
            
        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"Initialize state gets an error: {ex}")
            return ABORT
        
        else: return SUCCEED
    
class EndMission(State):

    def __init__(self) -> None:
        super().__init__(outcomes={SUCCEED})
        


    def execute(self, blackboard: Blackboard) -> str:

        try:
            yasmin.YASMIN_LOG_INFO("Ending state")

        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"Exception {ex}")
            return ABORT
        else: return SUCCEED