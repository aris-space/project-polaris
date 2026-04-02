import pandas as pd
import json
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def get_bag_data(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    data = []

    # Wir interessieren uns für diese 3 Topics
    target_topics = ["/pixhawk/scaled_pressure", "/ping_sonar/distance", "/imu/data"]

    print("Extrahiere Daten aus MCAP...")
    while reader.has_next():
        (topic, msg_serialized, t) = reader.read_next()
        if topic in target_topics:
            msg_type = get_message(topic_types[topic])
            msg = deserialize_message(msg_serialized, msg_type)

            # Basis-Eintrag mit Zeitstempel (Sekunden)
            entry = {"timestamp": t * 1e-9}

            if topic == "/pixhawk/scaled_pressure":
                entry["pressure"] = msg.fluid_pressure
            elif topic == "/ping_sonar/distance":
                try:
                    sonar_json = json.loads(msg.data)
                    entry["sonar_dist"] = float(sonar_json["distance"]) / 1000.0
                except:
                    continue
            elif topic == "/imu/data":
                # Wir nehmen die Orientierung (Quaternion) oder Beschleunigung
                entry["imu_roll"] = msg.orientation.x  # Nur als Beispiel
                entry["imu_pitch"] = msg.orientation.y
                # Falls du Position x/y willst: Die IMU gibt meist nur Beschleunigung!
                entry["accel_x"] = msg.linear_acceleration.x
                entry["accel_y"] = msg.linear_acceleration.y

            data.append(entry)

    return pd.DataFrame(data)


# --- Ausführung ---
path = "src/ice_estimates/ice_estimates/fixed_pool_test_2026_03_19-18_21_47"
df = get_bag_data(path)

# DER TRICK FÜR DIE SAUBERE TABELLE:
# 1. Nach Zeit sortieren
df = df.sort_values("timestamp")

# 2. Forward Fill: Fülle die Lücken mit dem jeweils letzten bekannten Wert auf
# So stehen Druck, Sonar und IMU in einer Zeile nebeneinander
df = df.ffill().dropna()

# 3. Optional: Nur jede 10. Zeile behalten, damit die CSV nicht riesig wird (10Hz)
df = df.iloc[::5, :]

df.to_csv("clean_pool_test.csv", index=False)
print(f"Fertig! Saubere Tabelle unter 'clean_pool_test.csv' gespeichert.")
