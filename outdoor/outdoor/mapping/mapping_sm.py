import logging
logging.basicConfig(level=logging.INFO)

import rclpy
import yasmin
from yasmin_ros.basic_outcomes import ABORT, RETRY, SUCCEED


from yasmin.state_machine import StateMachine

from outdoor.mapping.states import (InitializeMission, 
                                    EndMission, 
                                    OakdConfig, 
                                    FullMapping)


class MappingSM(StateMachine):


    def __init__(self)->None:
        super().__init__(outcomes={SUCCEED, ABORT})

        self.add_state("INITIALIZE MISSION",
                       InitializeMission(), 
                       {SUCCEED: "OAK-D CONFIG", ABORT: "INITIALIZE MISSION"},)
        self.add_state("OAK-D CONFIG", 
                       OakdConfig(),
                       {SUCCEED: "FULL_MAPPING", RETRY: "OAK-D CONFIG"})
        self.add_state("FULL_MAPPING", 
                       FullMapping(),
                       {SUCCEED:"END MISSION", ABORT: "OAK-D CONFIG"})
        self.add_state("END MISSION",
                       EndMission(),
                       {SUCCEED:SUCCEED})
        

def main() -> None:

    rclpy.init()

    try:
        bora = MappingSM()
        status = bora()
        yasmin.YASMIN_LOG_INFO(f"Mission status: {status}")

    except KeyboardInterrupt:...
    except Exception as ex: 
        print(f"Mission failed with exception: {ex}")

    finally:  
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
