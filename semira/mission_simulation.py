import csv
import random
import math
import os

# Bestimme den Ordner, in dem dieses Skript liegt
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, 'mission_data.csv')

# Parameter
duration_s = 2000  
hz = 2
total_steps = duration_s * hz
area_size = 100
lane_width = 5 

def generate_dynamic_mission():
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x_coord', 'y_coord', 'pressure_bar', 'dist_to_ice_m', 'roll_deg', 'pitch_deg', 'timestamp_s'])
        
        x, y = 0.0, 0.0
        direction = 1
        
        # Zeitfenster der Instabilität
        anomaly_start_s = 1800
        anomaly_duration_s = 20
        anomaly_end_s = anomaly_start_s + anomaly_duration_s

        for step in range(total_steps):
            t = step / hz
            
            # 1. Pfad-Logik (Rasenmäher)
            total_path_needed = (area_size / lane_width) * area_size
            speed_per_step = (total_path_needed / total_steps) * 1.2
            
            if direction == 1:
                x += speed_per_step
                if x >= area_size:
                    x = area_size
                    y += lane_width
                    direction = -1
            else:
                x -= speed_per_step
                if x <= 0:
                    x = 0
                    y += lane_width
                    direction = 1

            # 2. Sensor-Werte & Instabilität (20 Sekunden)
            if anomaly_start_s <= t <= anomaly_end_s:
                elapsed_anomaly = t - anomaly_start_s
                # Dämpfungsfaktor: Schwingt aus und beruhigt sich wieder
                recovery_factor = max(0, 1 - (elapsed_anomaly / anomaly_duration_s))
                
                roll = math.sin(t * 2) * 45 * recovery_factor + random.uniform(-5, 5)
                pitch = math.cos(t * 1.5) * 30 * recovery_factor + random.uniform(-3, 3)
                pressure = 1.1 + (math.sin(t) * 0.1 * recovery_factor)
                dist_to_ice = 0.6 + (math.cos(t) * 0.2 * recovery_factor)
            else:
                roll = random.uniform(-1.2, 1.2)
                pitch = random.uniform(-0.8, 0.8)
                pressure = 1.1 + random.uniform(-0.005, 0.005)
                dist_to_ice = 0.6 + random.uniform(-0.02, 0.02)
            
            y_clamped = min(y, area_size)
            
            writer.writerow([round(x, 2), round(y_clamped, 2), round(pressure, 3), 
                             round(dist_to_ice, 2), round(roll, 2), round(pitch, 2), t])

    print(f"Datei erfolgreich erstellt unter: {file_path}")

if __name__ == "__main__":
    generate_dynamic_mission()