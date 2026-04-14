import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Int32
class ArmStatusSubscriber(Node):

    def __init__(self): 
        super().__init__('arm_status_subscriber')

        self.last_shoulder  = None 
        self.last_elbow = None 
        self.last_gripper = None 

        self.create_subscription(
            Int32,
            '/arm/shoulder_angle',
            self.shoulder_callback,
            10            
        )
        self.create_subscription(
            Int32,
            '/arm/elbow_angle',
            self.elbow_callback,
            10            
        )
        self.create_subscription(
            Int32,
            '/arm/gripper_angle',
            self.gripper_callback,
            10
        )

    
    def shoulder_callback(self, msg):
        angle = msg.data 

        if self.last_shoulder is None:
            self.last_shoulder = angle 
            print(f'[UPDATE] Shoulder Angle: {angle} deg')
            return
        
        delta  = angle - self.last_shoulder
        if delta == 0:
            return  
        
        direction = 'Pitch UP' if delta  > 0 else "Pitch DOWN"

        self.last_shoulder = angle

    
    def ellbow_callback(self, msg):
        angle = msg.data 

        if self.last_elbow is None:
            self.last_elbow = angle 
            print(f'Elbow Angle: {angle} deg')
            return
        
        delta  = angle - self.last_elbow
        if delta == 0:
            return  
        
        direction = 'Pitch UP' if delta  > 0 else "Pitch DOWN"

        self.last_elbow = angle

    def gripper_callback(self, msg):
        angle = msg.data 

        if self.last_gripper is None:
            self.last_gripper = angle 
            print(f'Gripper Angle: {angle} deg')
            return
        
        delta  = angle - self.last_gripper
        if delta == 0:
            return  
        
        direction = 'OPENING' if delta  > 0 else "CLOSING"

        self.last_gripper = angle

def main(args=None):
    rclpy.init(args=args)
    node = ArmStatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n  [EXIT] Subscriber stopped by user (Ctrl+C).')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()