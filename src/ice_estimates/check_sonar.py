import mcap
from mcap.reader import make_reader

# Pfad zu deiner reparierten Datei
file_path = "src/ice_estimates/ice_estimates/fixed_pool_test_2026_03_19-18_21_47"

print(f"Suche Sonar-Daten in {file_path}...")

count = 0
with open(file_path, "rb") as f:
    reader = make_reader(f)
    for schema, channel, message in reader.iter_messages():
        if channel.topic == "/ping_sonar/distance":
            # Wir drucken die Rohdaten (HEX) und versuchen sie als Text zu interpretieren
            try:
                # ROS2 Strings haben oft einen 4-Byte Header für die Länge
                # Wir dekodieren ab Byte 4, um den reinen Text zu sehen
                raw_text = message.data[4:].decode("utf-8", errors="ignore")
                print(
                    f"Nachricht {count}: Roh-Daten: {message.data.hex()} | Text-Interpretation: '{raw_text}'"
                )
            except:
                print(
                    f"Nachricht {count}: Konnte nicht dekodieren. Roh: {message.data.hex()}"
                )

            count += 1
            if count >= 20:  # Wir schauen uns nur die ersten 20 an
                break

if count == 0:
    print("KEINE Daten auf /ping_sonar/distance gefunden!")
