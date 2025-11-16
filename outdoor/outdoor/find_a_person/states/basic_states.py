import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin.state import State
from yasmin.blackboard import Blackboard

from yasmin_ros.basic_outcomes import SUCCEED, ABORT, RETRY

from mirela_sdk.control.mavros.mavros_api import MavDrone

from outdoor.find_a_person.constants import (TAKE_OFF_ALTITUDE, 
                                             DETECTION_ALTITUDE, 
                                             TARGET_COORDINATE,
                                             INIT_MISSION_ATTEMPTS)


class InitializeMission(State):

    def __init__(self)-> None:

        super().__init__(outcomes={SUCCEED, RETRY, ABORT})

        self.node = YasminNode.get_instance()
        self.drone = MavDrone(self.node, False)
        self.init_attempts = 0


    def execute(self, blackboard: Blackboard) -> str:
        
        
        blackboard["drone"] = self.drone
        blackboard['camera_crash_state'] = None
        self.drone.check_driver_node(2.0)
        lat, long = TARGET_COORDINATE

        if self.init_attempts == INIT_MISSION_ATTEMPTS: 
            yasmin.YASMIN_LOG_ERROR(f"Multiple attempts to initialize mission: ABORTING")
            return ABORT

        try:

            if not self.drone.get_state.armed: self.drone.arm_takeoff(TAKE_OFF_ALTITUDE)

            yasmin.YASMIN_LOG_INFO("Going to the target coordinate")
            heading = self.drone.gps_controller.calculate_bearing(lat, long)
            self.drone.offboard_position_gps_coords(latitude = lat, 
                                                    longitude = long, 
                                                    heading= heading,
                                                    strategy = "mavros")
            

            yasmin.YASMIN_LOG_INFO("Descending the drone to detection altitude")
            altitude = self.drone.get_gps.altitude - (TAKE_OFF_ALTITUDE - DETECTION_ALTITUDE)
            self.drone.offboard_position_gps_coords(lat, long, altitude, strategy="mavros")


        except Exception as ex:
            self.init_attempts += 1
            yasmin.YASMIN_LOG_ERROR(f"Initialize state gets an error (RETRYING): {ex}")
            return RETRY

        else: return SUCCEED


class ReturnToLaunch(State):

    def __init__(self):
        super().__init__(outcomes = {SUCCEED})

    def execute(self, blackboard: Blackboard):

        yasmin.YASMIN_LOG_ERROR("Mission did not finish. Trying to return to home")
        
        self.drone: MavDrone = blackboard['drone']
        
        try:self.drone.rtl(TAKE_OFF_ALTITUDE, rtl_strategy="default")
        except Exception as ex: 
            yasmin.YASMIN_LOG_ERROR(f"RTL state gets an error: {ex}")
            return
        
        else: return  SUCCEED


class EndMission(State):

    def __init__(self) -> None:
        super().__init__(outcomes={SUCCEED})

    def execute(self, blackboard: Blackboard) -> str:

        self.drone: MavDrone = blackboard['drone']
        try:
            yasmin.YASMIN_LOG_INFO("Ending state")
            if self.drone.get_state.armed: self.drone.land()

        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"Exception {ex}")
            return 
        else: return SUCCEED