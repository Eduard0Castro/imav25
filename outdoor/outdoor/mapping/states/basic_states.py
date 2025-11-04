import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin.blackboard import Blackboard
from yasmin.state import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, RETRY

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing import OakdCam

from outdoor.mapping.constants import ALTITUDE


class InitializeMission(State):

    def __init__(self) -> None:

        super().__init__(outcomes = {SUCCEED, ABORT})

        self.drone = MavDrone(YasminNode.get_instance())


    def execute(self, blackboard: Blackboard) -> str:

        blackboard['drone'] = self.drone

        self.drone.arm_takeoff(ALTITUDE)
        self.drone.delay(30.0)

        return SUCCEED


class OakdConfig(State):

    def __init__(self) -> None:

        super().__init__(outcomes = {SUCCEED, RETRY})

    def execute(self, blackboard: Blackboard) -> None:
        try:
            self.oakd_cam = OakdCam()
            self.oakd_cam.start()

        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"Oakd config error: {ex}")
            return RETRY
        else: 
            blackboard[ 'oakd']  = self.oakd_cam
            return SUCCEED
        

class EndMission(State):

    def __init__(self) -> None:
        super().__init__(outcomes = {SUCCEED})


    def execute(self, blackboard: Blackboard) -> str:
        
        try:
            oakd:OakdCam = blackboard['oakd']
            oakd.close()
            drone: MavDrone = blackboard['drone']
            drone.rtl(ALTITUDE, rtl_strategy="mavros")

        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"End Mission exception: {ex}")
        else: return SUCCEED
