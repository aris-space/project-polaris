import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import String
import mcap
from mcap.stream_reader import StreamReader
from rclpy.serialization import deserialize_message
import time

class ManualPlayer(Node):
    def __init__(self):
        super().__init__('manual_player')
        self.p_pub = self.create_publisher(FluidPressure, '/pixhawk/scaled_pressure', 10)
        self.i_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.s_pub = self.create_publisher(String, '/ping_sonar/distance', 10)

    def play(self, file_path):
        self.get_logger().info(f"Erzwinge lineares Lesen von: {file_path}")
        
        with open(file_path, "rb") as f:
            # StreamReader ist robuster gegen korrupte Dateien als make_reader
            reader = StreamReader(f)
            
            channels = {}
            for record in reader.records():
                # 1. Kanäle (Topics) registrieren
                if isinstance(record, mcap.records.Channel):
                    channels[record.id] = record.topic
                
                # 2. Nachrichten verarbeiten
                elif isinstance(record, mcap.records.Message):
                    topic = channels.get(record.channel_id)
                    
                    try:
                        if topic == '/pixhawk/scaled_pressure':
                            msg = deserialize_message(record.data, FluidPressure)
                            self.p_pub.publish(msg)
                        elif topic == '/imu/data':
                            msg = deserialize_message(record.data, Imu)
                            self.i_pub.publish(msg)
                        elif topic == '/ping_sonar/distance':
                            msg = deserialize_message(record.data, String)
                            self.s_pub.publish(msg)
                        
                        # Kleiner Sleep (0.5ms) für Stabilität
                        time.sleep(0.0005)
                    except Exception:
                        # Überspringe einzelne kaputte Nachrichten
                        continue

def main():
    rclpy.init()
    player = ManualPlayer()
    try:
        player.play("src/ice_estimates/ice_estimates/bag_pool_test_2026_03_19-18_21_47_0.mcap")
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Abbruch durch Fehler: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()