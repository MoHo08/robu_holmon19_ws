# Datein importieren fürs senden und empfangen von Ros Datein ................................................
import rclpy                                                        #Ros schnittstelle für Python
from rclpy.node import Node
#.............................................................................................................

# Klasse erstellen ...........................................................................................
class MinimalParameter(Node):
    #++++ Konstruktor ++++
    def __init__(self):
        # super: auf Elternklasse zugreifen
        # Konstruktor der Elternklasse(NODE) aufrufen und Namen der Node vergeben
        super().__init__('MinimalParameter')

        # Parameter Beschreibung
        from rcl_interfaces.msg import ParameterDescriptor
        my_parameter_description = ParameterDescriptor(description='This Parameter is mine!')

        # Parameter erstellen:      name        wert            beschreibung
        self.declare_parameter('my_parameter', 'work', my_parameter_description)

        # Parameter lesen
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        print("my_parameter: ", my_param)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').value

        print("my_parameter: ", my_param)
        # self.get_logger().info('Hello %s!', my_param)
        
#.............................................................................................................

# Main .......................................................................................................
def main():
    rclpy.init()

    node = MinimalParameter()

    rclpy.spin(node)
#.............................................................................................................

if __name__ == '__main__':
    main()