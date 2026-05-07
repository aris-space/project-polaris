import mcap
from mcap.reader import make_reader
from mcap.writer import Writer

input_file = "src/ice_estimates/ice_estimates/bag_pool_test_2026_03_19-18_21_47_0.mcap"
output_file = "fixed.mcap"

print(f"Repariere {input_file}...")

with open(input_file, "rb") as f:
    reader = make_reader(f)
    with open(output_file, "wb") as out:
        writer = Writer(out)
        # KORREKTUR HIER:
        writer.start(profile="ros2")

        channels = {}
        for schema, channel, message in reader.iter_messages():
            if channel.id not in channels:
                channels[channel.id] = writer.register_channel(
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                    schema_id=writer.register_schema(
                        name=schema.name, encoding=schema.encoding, data=schema.data
                    ).id,
                )
            writer.add_message(
                channel_id=channels[channel.id],
                log_time=message.log_time,
                data=message.data,
                publish_time=message.publish_time,
            )
        writer.finish()
print(f"Fertig! Datei erstellt: {output_file}")
