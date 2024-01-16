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

        # Parameter erstellen
        self.declare_parameter('my_parameter')

        # Parameter lesen
        my_param = self.get_parameter('my_parameter')

        print("my_parameter: ", my_param)
#.............................................................................................................

# Main .......................................................................................................
def main():
    rclpy.init()

    node = MinimalParameter()

    rclpy.spin(node)
#.............................................................................................................

if __name__ == '__main__':
    main()