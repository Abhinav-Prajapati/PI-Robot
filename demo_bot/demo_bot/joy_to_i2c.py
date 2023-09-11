import smbus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class SMBusNode(Node):

    def __init__(self):
        super().__init__('smbus_node')

        timer_period = 0.05 # sec
        self.BUS_DATA = [0, 0, 0, 0, 0,0,0,0]
        self.AXES_VALUE = [0,0,0]
        self.BUTTON_VALUE = [0,0,0,0]

        self.device_address = 0x09
        self.i2c_bus = smbus.SMBus(1) 

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(timer_period, self.callback_send_data_to_i2c)
        
    def is_drive_stop(self):
        '''
        Returs False if drive is moving ie. velX,velY,velW != 0
        '''
        if all(element == 0 for element in self.AXES_VALUE):
            return True
        else:
            return False

    def button_to_byte(self,button_arr:list[int])->int:
        '''
        Converts the list of given array of size 8 of element 0s and 1s 
        and assuming it a 1 byte binary number converts it to a decimal number 
        and sends it to bus date
        '''
        button_states = 0
        for i in button_arr:
            button_states <<= 1
            button_states |= int(i)
        return button_states

    def split_byte(self,num:int) -> (int,int):
        '''
        We can only send 1 byte of data to i2c bus.
        To send a data of multiple bytes split the byte in single byte
        ie (555) dec = ( 0000 0010 0010 1011 ) bin => (0000 0010) , (0010 1011)
        '''
        num = int((num+1)*1023)# mapping data form -1 to +1 to +0 to +1023
        high_bit=(num>>8) & 0xff
        low_bit=num & 0xff
        return (high_bit,low_bit)

    def joy_callback(self, msg:list[int]) -> None:
        self.AXES_VALUE[0]=msg.axes[1] # throtl
        self.AXES_VALUE[1]=-msg.axes[3] # Left / Right
        self.AXES_VALUE[2]=msg.axes[0] # Rotation

        self.BUTTON_VALUE[0]=msg.buttons[0] # PS4 Cross
        self.BUTTON_VALUE[1]=msg.buttons[1] # PS4 Circle
        self.BUTTON_VALUE[2]=msg.buttons[2] # PS4 Triangle
        self.BUTTON_VALUE[3]=msg.buttons[3] # PS4 Square

    def callback_send_data_to_i2c(self) -> None:
        """
        Sends data to an I2C device, formatting and preparing the data based on axes values and drive status.

        This method populates a data array, which is then sent to the specified I2C device. It splits 2-byte values
        into individual bytes, handles LED color changes based on drive status, and logs the sent data.

        Raises:
            IOError: If the I2C communication encounters an error, indicating a disconnected DriveHub.

        Returns:
            None
        """
        try:
            self.BUS_DATA[0],self.BUS_DATA[1]=self.split_byte(self.AXES_VALUE[0])
            self.BUS_DATA[2],self.BUS_DATA[3]=self.split_byte(self.AXES_VALUE[1])
            self.BUS_DATA[4],self.BUS_DATA[5]=self.split_byte(self.AXES_VALUE[2])
            self.BUS_DATA[6] = self.button_to_byte(self.BUTTON_VALUE)

            if self.is_drive_stop():
                self.BUS_DATA[7] = 4 # Green when drive is not moving
            else :
                self.BUS_DATA[7] = 6 # Purple when drive is moving 

            self.i2c_bus.write_i2c_block_data(self.device_address, 0, self.BUS_DATA)
            self.get_logger().info(f"Data sent to I2C device. {self.BUS_DATA}")

        except IOError as e :
            self.get_logger().info(f"DriveHub Disconnected")

    def is_connected(self) -> bool:
        try:
            self.i2c_bus.read_byte(self.device_address)
            self.get_logger().info(f"Data sent to I2C conncted")
            return True
        except IOError as e:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = SMBusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
