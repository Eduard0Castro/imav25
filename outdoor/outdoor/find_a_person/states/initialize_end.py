import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin.state import State
from yasmin.blackboard import Blackboard

from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone

class InitializeMission(State):

    def __init__(self)-> None:

        super().__init__(outcomes={SUCCEED, ABORT})

        self.node = YasminNode.get_instance()
        self.drone = MavDrone(self.node, False)


    def execute(self, blackboard: Blackboard) -> str:
        
        blackboard["drone"] = self.drone
        self.drone.check_driver_node(2.0)

        self.drone.arm_takeoff(10.0)
        #self.drone.offboard_gps_position()

        return SUCCEED
    
class EndMission(State):

    def __init__(self) -> None:
        super().__init__(outcomes={SUCCEED})
        


    def execute(self, blackboard: Blackboard) -> str:

        try:
            yasmin.YASMIN_LOG_INFO("Ending state")
            # drone: MavDrone = blackboard['drone']
            # drone.rtl(10)

        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"Exception {ex}")
            return ABORT
        else: return SUCCEED


        
        
        






