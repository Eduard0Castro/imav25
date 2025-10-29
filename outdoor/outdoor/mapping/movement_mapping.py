import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, 
                       QoSDurabilityPolicy, 
                       QoSReliabilityPolicy, 
                       QoSHistoryPolicy)

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mavros_msgs.msg import LogData

from outdoor.mapping.constants import COORDINATES


class MovementMapping(Node):

    def __init__(self) -> None:

        super().__init__("movement_mapping_node")
        self.qos_profile = QoSProfile(history =     QoSHistoryPolicy.KEEP_ALL,  
                                      durability =  QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                      reliability = QoSReliabilityPolicy.RELIABLE,)
        self.altitude = 60.0

        
        self.coordinates = COORDINATES
        

        self.drone = MavDrone(self, False)
        self.msg = LogData()
        self.msg.id = 0


        self.photo_pub = self.create_publisher(LogData, "/photos/controller", self.qos_profile,)
        self.stopped_mapping_pub = self.create_publisher(LogData, "/photos/controller_stopped", 
                                                         self.qos_profile,)

        self.get_logger().info("Movement Mapping has been initialized")


    def run(self) -> None:

        self.drone.arm_takeoff(self.altitude)
        self.drone.delay(17.0)
        self.photo_pub.publish(msg=self.msg)
        self.drone.delay(3.0)
        self.msg.id = 12

        
        for lat, long in self.coordinates:
            heading = self.drone.gps_controller.calculate_bearing(lat, long)
            self.drone.offboard_position_gps_coords(lat, long, heading=heading, strategy="mavros")
            self.drone.delay(2.0)
            self.stopped_mapping_pub.publish(self.msg)
            self.drone.delay(1.0)
        
        self.drone.rtl(int(self.altitude), rtl_strategy="AP")
        
        self.msg.id = 10
        self.photo_pub.publish(self.msg)

        

def main(args = None):

    rclpy.init(args=args)
    movement = MovementMapping()
    try:
        movement.run()
        rclpy.spin(movement)
    except KeyboardInterrupt:...
    except Exception as ex:print(f"Exception: {ex}")
    finally: movement.destroy_node()
    

if __name__ == "__main__":
    main()