#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String
import time

class ServoPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('servo_publisher', anonymous=True)
        
        # Create publisher for gate status
        self.pub = rospy.Publisher('/gate/status', String, queue_size=10)
        
        # Serial port configuration
        self.serial_port = '/dev/ttyUSB0'  # Ubah sesuai port Arduino (Windows: COM3, COM4, etc)
        self.baud_rate = 115200
        self.ser = None
        
        # Connect to Arduino
        self.connect_serial()
        
    def connect_serial(self):
        """Connect to Arduino via serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo(f"Connected to {self.serial_port} at {self.baud_rate} baud")
            time.sleep(2)  # Wait for Arduino to reset
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to serial port: {e}")
            rospy.signal_shutdown("Serial connection failed")
    
    def run(self):
        """Main loop to read from serial and publish"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.in_waiting > 0:
                    # Read data from Arduino
                    data = self.ser.readline().decode('utf-8').strip()
                    
                    # Check for valid messages
                    if data in ['GATE_OPEN', 'GATE_CLOSED']:
                        rospy.loginfo(f"Received: {data}")
                        self.pub.publish(data)
                    
            except UnicodeDecodeError:
                rospy.logwarn("Failed to decode serial data")
            except Exception as e:
                rospy.logerr(f"Error: {e}")
            
            rate.sleep()
    
    def shutdown(self):
        """Cleanup on shutdown"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        rospy.loginfo("Serial connection closed")

if __name__ == '__main__':
    try:
        publisher = ServoPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        publisher.shutdown()