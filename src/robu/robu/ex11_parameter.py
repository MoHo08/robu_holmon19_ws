# Parameter sind Variablen, die veränderbar sind, während das Programm läuft

# Parameter im Terminal
# ros2 param list .... gibt alle parameter aus
# ros2 param get <Node> <Parameter> .... git den Wert des Parameters aus
# ros2 param set <Node> <Parameter> <Wert> .... setzt den Wert des Parameters



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

        # Parameter erstellen:      name        wert         beschreibung
        self.declare_parameter('my_parameter', 0.1, my_parameter_description)

        # mehrere Parameter erstellen
        self.declare_parameters(namespace='', 
                                parameters=[('forward_speed_wf_slow', 0.05),
                                            ('forward_speed_wf_fast', 0.1),
                                            ('turning_speed_wf_slow', 0.1),
                                            ('turning_speed_wf_fast', 1.0),
                                            ('dist_thresh_wf', 0.3),
                                            ('dist_hysteresis_wf', 0.02),])

        #++++ Parameter lesen ++++
        #++++ gibt das Objekt des Parameters zurück ++++
        #   my_param = self.get_parameter('my_parameter')
        #++++ gibt den Wert des Parameters zurück ++++
        #   my_param = self.get_parameter('my_parameter').get_parameter_value().string_value 
        #   my_param = self.get_parameter('my_parameter').value

        self.timer = self.create_timer(1, self.timer_callback)


    def timer_callback(self):
        # my_param = self.get_parameter('my_parameter').value
        # print("my_parameter: ", my_param)
        # self.get_logger().info('Hello %s!', my_param)

        print("forward_speed_wf_slow: ", self.get_parameter('forward_speed_wf_slow').value)
        print("forward_speed_wf_fast: ", self.get_parameter('forward_speed_wf_fast').value)
        print("turning_speed_wf_slow: ", self.get_parameter('turning_speed_wf_slow').value)
        print("turning_speed_wf_fast: ", self.get_parameter('turning_speed_wf_fast').value)
        print("dist_thresh_wf:        ", self.get_parameter('dist_thresh_wf').value)
        print("dist_hysteresis_wf:    ", self.get_parameter('dist_hysteresis_wf').value)
        print("")
        
#.............................................................................................................

# Main .......................................................................................................
def main():

    rclpy.init()

    node = MinimalParameter()

    rclpy.spin(node)
#.............................................................................................................

if __name__ == '__main__':
    main()