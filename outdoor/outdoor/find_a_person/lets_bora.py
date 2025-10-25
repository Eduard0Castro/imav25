import logging
logging.basicConfig(level=logging.INFO)

import rclpy
from yasmin import StateMachine

from yasmin_ros.basic_outcomes import SUCCEED, ABORT
import yasmin

from outdoor.find_a_person.states import (OakdConfig, 
                                               MoveToPerson,
                                               EndMission)


class LetsBoraSM(StateMachine):

    def __init__(self, )-> None:

        super().__init__(outcomes={SUCCEED, ABORT})

        self.add_state(
            name="OAK-D CONFIG",
            state = OakdConfig(),
            transitions={SUCCEED: "MOVE TO PERSON", ABORT: "OAK-D CONFIG"},

        )

        self.add_state(
            name = "MOVE TO PERSON", 
            state = MoveToPerson(),
            transitions={SUCCEED: "END", ABORT: "OAK-D CONFIG"}
        )

        self.add_state(
            name = "END",
            state = EndMission(),
            transitions = {SUCCEED:SUCCEED}
        )

def main() -> None:

    rclpy.init()

    try:
        bora = LetsBoraSM()
        status = bora()
        yasmin.YASMIN_LOG_INFO(f"Mission status: {status}")

    except Exception as ex: 
        print(f"Mission failed with exception: {ex}")

    finally:  
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()