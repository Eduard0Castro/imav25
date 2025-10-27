import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, 
                       QoSDurabilityPolicy, 
                       QoSReliabilityPolicy, 
                       QoSHistoryPolicy)

from mavros_msgs.msg import LogData

from mirela_sdk.image_processing.camera.image_handler import ImageHandler

from pathlib import Path


class PhotosMapping(Node):

    def __init__(self) -> None:
        
        super().__init__("photos_mapping_node")

        self.path = Path(__file__).resolve().parent
        self.image_handler = ImageHandler(self, "oakd", self.take_photos)
        self.qos_profile = QoSProfile(history     =           QoSHistoryPolicy.KEEP_ALL,
                                      durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                      reliability =       QoSReliabilityPolicy.RELIABLE,)
        
        self.photo_sub = self.create_subscription(LogData, 
                                                  "/photos/controller", 
                                                  self.photo_sub_callback, 
                                                  self.qos_profile,)
        
        self.stopped_photos_sub = self.create_subscription(LogData, 
                                                  "/photos/controller_stopped", 
                                                  lambda data: self.__setattr__("stopped", 
                                                                                data.id),
                                                  self.qos_profile,
        )

        self.photo_action = {0: self.image_handler.run, 
                             10: self.image_handler.cleanup}
        self.photos = 0
        self.stopped = 0
        self.cleaned = False

        self.get_logger().info("Photos Mapping node has been initialized")


    def photo_sub_callback(self, msg: LogData) -> None:
        
        action = self.photo_action.get(msg.id, None)
        if action is not None: action()


    def take_photos(self, img: cv2.Mat) -> None:

        if self.stopped == 12:
            self.photos = int(self.photos) + 1
            if self.photos < 10: self.photos = '0' + str(self.photos)
            cv2.imwrite(f"{self.path}/mapping_photos/photo_{self.photos}.jpg", img)
            
            self.get_logger().info(f"Photo {self.photos}")
            self.stopped = 0
        
    def cleanup(self) -> None:

        if not self.cleaned:
            
            self.image_handler.cleanup()
            self.destroy_node()
            self.cleaned = True
    

def main(args = None):
    rclpy.init(args = args)

    try:
        photos = PhotosMapping()
        rclpy.spin(photos)

    except KeyboardInterrupt:...
    except Exception as ex: print(f"Exception: {ex}")
    finally:
        photos.cleanup()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()