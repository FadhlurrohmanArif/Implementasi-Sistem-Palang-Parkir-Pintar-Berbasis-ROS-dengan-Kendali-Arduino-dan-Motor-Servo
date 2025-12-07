import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BarrierNode(Node):
    def __init__(self):
        super().__init__('barrier_node')
        
        # Create subscription to gate_status topic
        self.subscription = self.create_subscription(
            String,
            'gate_status',
            self.topic_callback,
            10
        )
        
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Barrier Node initialized and listening to /gate_status')
    
    def topic_callback(self, msg):
        """Callback function to handle incoming messages"""
        self.get_logger().info(f'Status Palang: "{msg.data}"')
        
        # Handle different states
        if msg.data == 'GATE_OPEN':
            self.handle_gate_open()
        elif msg.data == 'GATE_CLOSED':
            self.handle_gate_closed()
    
    def handle_gate_open(self):
        """Actions when gate is opened"""
        self.get_logger().warn('⚠️ GATE OPENED - Vehicle can pass')
        # Add your logic here (beep, light, etc)
    
    def handle_gate_closed(self):
        """Actions when gate is closed"""
        self.get_logger().info('✓ GATE CLOSED - Barrier is locked')
        # Add your logic here

def main(args=None):
    rclpy.init(args=args)
    barrier_node = BarrierNode()
    
    try:
        rclpy.spin(barrier_node)
    except KeyboardInterrupt:
        pass
    finally:
        barrier_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    main()
