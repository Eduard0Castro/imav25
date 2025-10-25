from pathlib import Path

from mirela_sdk.image_processing.camera.oakd_cam import OakdCam

from outdoor.find_a_person.constants import LABELS
import yasmin
from yasmin.blackboard import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

class OakdConfig(State):

    def __init__(self) -> None:

        super().__init__(outcomes={SUCCEED, ABORT})


    def __init_oakd(self) -> None:

        self.oakd = OakdCam()

        model_path = Path(__file__).parent.resolve().absolute() / "blob/ssd_mobile_net.blob"
        self.labels = LABELS

        self.oakd.create_spatial_detection_network(model_path, self.labels)
        self.oakd.init_cam(usb2_mode=True)

    def __set_queues(self) -> None:

        self.__preview_queue = self.oakd.getQueue("rgb", maxSize=4, blocking=False)
        self.__detection_nnqueue = self.oakd.getQueue("detections", maxSize=4, blocking=False)
        self.oakd.getQueue("depth", maxSize=4, blocking=False)

    def execute(self, blackboard: Blackboard) -> str:

        yasmin.YASMIN_LOG_INFO("Executing OAK-D initial configs")

        try:
            self.__init_oakd()
            self.__set_queues()

        except Exception as ex:
            yasmin.YASMIN_LOG_ERROR(f"OAK-D config gets an error: {ex}")
            return ABORT
        else: 
            
            blackboard['oakd_object'] = self.oakd
            blackboard['detection_nnqueue'] = self.__detection_nnqueue
            blackboard['preview_queue'] = self.__preview_queue

            blackboard['labels'] = self.labels

            return SUCCEED
