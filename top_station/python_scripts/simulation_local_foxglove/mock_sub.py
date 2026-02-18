"""
Mock Submarine Telemetry Simulator for Foxglove Studio

This module simulates a remotely operated vehicle (ROV) submarine and publishes
sensor data and 3D visualization to Foxglove Studio via WebSocket on port 8765.
"""
# hello world
import asyncio
import time
import math
import json
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener


# =============================================================================
# SYSTEM PARAMETERS
# =============================================================================
# Configuration values for the ROV (Remotely Operated Vehicle)

params = {
    "p_gain": 1.5,                 # Proportional gain for PID controller
    "i_gain": 0.05,                # Integral gain for PID controller
    "d_gain": 0.2                  # Derivative gain for PID controller
}




# =============================================================================
# EVENT LISTENER
# =============================================================================
# Handles incoming client messages and parameter updates from Foxglove Studio

class ROVListener(FoxgloveServerListener):
    """Listens for client interactions and parameter changes from Foxglove."""
    
    def __init__(self):
        self.state_index = 0  # Tracks current mode index for cycling through states
    
    async def on_client_message(self, server, client_channel_id, payload):
        """Called when client sends a message (button click, etc.)"""
        states = ["MANUAL", "PRO"]
        # Cycle through modes each time a message is received
        self.state_index = (self.state_index + 1) % len(states)
        # Note: mode is now only a ROS topic, not a parameter

    
    async def on_set_parameters(self, server, parameter_updates):
        """Called when parameters are updated from the Foxglove UI."""
        for update in parameter_updates:
            if update["name"] in params:
                params[update["name"]] = update["value"]
        # Sync all parameters back to the client
        await server.update_parameters([{"name": k, "value": v} for k, v in params.items()])


async def _register_channels(server):
    """Register all topics/channels with the Foxglove server."""
    
    # Define reusable JSON schema for depth/distance measurements
    num_schema = {"type": "object", "properties": {"meters": {"type": "number"}}}
    
    # Diagnostic schema for system health and status
    diag_schema = {
        "type": "object",
        "properties": {
            "header": {
                "type": "object",
                "properties": {
                    "stamp": {"type": "object", "properties": {"sec": {"type": "number"}, "nsec": {"type": "number"}}},
                    "frame_id": {"type": "string"}
                }
            },
            "status": {"type": "array", "items": {"type": "object"}}
        }
    }
    
    # Dictionary to store all channels for easy access
    channels = {}
    


    # =========================================================================
    # SENSOR TOPICS - Environment/Depth measurements
    # =========================================================================
    
    # Control mode topic
    channels["mode"] = await server.add_channel({
        "topic": "/control/mode",
        "encoding": "json",
        "schemaName": "Mode",
        "schema": json.dumps({"type": "object", "properties": {"mode": {"type": "string"}}})
    })
    
    channels["ice_depth"] = await server.add_channel({
        "topic": "/sensor/ice_depth",
        "encoding": "json",
        "schemaName": "Depth",
        "schema": json.dumps(num_schema)
    })
    
    channels["sub_depth"] = await server.add_channel({
        "topic": "/sensor/sub_depth",
        "encoding": "json",
        "schemaName": "Depth",
        "schema": json.dumps(num_schema)
    })
    
    channels["target_depth"] = await server.add_channel({
        "topic": "/sensor/target_depth",
        "encoding": "json",
        "schemaName": "Depth",
        "schema": json.dumps(num_schema)
    })
    
    channels["bottom_depth"] = await server.add_channel({
        "topic": "/sensor/bottom_depth",
        "encoding": "json",
        "schemaName": "Depth",
        "schema": json.dumps(num_schema)
    })
    
    channels["front_dist"] = await server.add_channel({
        "topic": "/sensor/front_dist",
        "encoding": "json",
        "schemaName": "Dist",
        "schema": json.dumps(num_schema)
    })
    # Battery level topic
    battery_schema = {"type": "object", "properties": {"percentage": {"type": "number"}}}
    channels["battery_level"] = await server.add_channel({
        "topic": "/system/battery_level",
        "encoding": "json",
        "schemaName": "BatteryLevel",
        "schema": json.dumps(battery_schema)
    })
    
    # =========================================================================
    # ACTUATOR TOPICS - Thruster Control
    # =========================================================================
    
    # Define RPM schema
    rpm_schema = {"type": "object", "properties": {"rpm": {"type": "number"}}}
    
    channels["thruster_1_rpm"] = await server.add_channel({
        "topic": "/actuator/thruster_1_rpm",
        "encoding": "json",
        "schemaName": "RPM",
        "schema": json.dumps(rpm_schema)
    })
    
    channels["thruster_2_rpm"] = await server.add_channel({
        "topic": "/actuator/thruster_2_rpm",
        "encoding": "json",
        "schemaName": "RPM",
        "schema": json.dumps(rpm_schema)
    })
    
    channels["thruster_3_rpm"] = await server.add_channel({
        "topic": "/actuator/thruster_3_rpm",
        "encoding": "json",
        "schemaName": "RPM",
        "schema": json.dumps(rpm_schema)
    })
    
    channels["thruster_4_rpm"] = await server.add_channel({
        "topic": "/actuator/thruster_4_rpm",
        "encoding": "json",
        "schemaName": "RPM",
        "schema": json.dumps(rpm_schema)
    })
    
    channels["thruster_5_rpm"] = await server.add_channel({
        "topic": "/actuator/thruster_5_rpm",
        "encoding": "json",
        "schemaName": "RPM",
        "schema": json.dumps(rpm_schema)
    })
    
    channels["thruster_6_rpm"] = await server.add_channel({
        "topic": "/actuator/thruster_6_rpm",
        "encoding": "json",
        "schemaName": "RPM",
        "schema": json.dumps(rpm_schema)
    })


    # =========================================================================
    # LOCALIZATION TOPICS - Position and Navigation
    # =========================================================================
    
    channels["gps"] = await server.add_channel({
        "topic": "/sensor/gps",
        "encoding": "json",
        "schemaName": "foxglove.LocationFix"
    })
    
    channels["pose"] = await server.add_channel({
        "topic": "/sensor/pose",
        "encoding": "json",
        "schemaName": "foxglove.Pose"
    })
    
    channels["tf"] = await server.add_channel({
        "topic": "/tf",
        "encoding": "json",
        "schemaName": "foxglove.FrameTransforms"
    })
    


    # =========================================================================
    # VISUALIZATION TOPICS - 3D Scene and Models
    # =========================================================================
    
    channels["scene"] = await server.add_channel({
        "topic": "/sensor/model",
        "encoding": "json",
        "schemaName": "foxglove.SceneUpdate"
    })
    

    
    # =========================================================================
    # SYSTEM TOPICS - Diagnostics and Status
    # =========================================================================
    
    channels["diagnostics"] = await server.add_channel({
        "topic": "/diagnostics",
        "encoding": "json",
        "schemaName": "diagnostic_msgs/DiagnosticArray",
        "schema": json.dumps(diag_schema)
    })
    
    return channels


async def main():
    """Main simulation loop."""
    
    # Initialize Foxglove WebSocket server
    async with FoxgloveServer(
        "0.0.0.0",
        8765,
        "Polaris-Alpha",
        capabilities=["clientPublish", "parameters"]
    ) as server:
        # Setup event listener
        listener = ROVListener()
        server.set_listener(listener)
        
        # Register all channels/topics
        channels = await _register_channels(server)
        
        # Initialize parameters on the server
        await server.update_parameters([{"name": k, "value": v} for k, v in params.items()])
        
        # Starting GPS coordinates (Zurich area) and simulation time
        lat, lon, t = 47.3450, 8.5650, 0
        
        # Mode and target depth - published as ROS topics, not parameters
        mode = "MANUAL"
        target_depth = -12.5
        
        # Main simulation loop - runs indefinitely, publishing at 10 Hz
        while True:
            t += 0.05  # Advance simulation time
            now = time.time()
            now_ns = time.time_ns()

            # =====================================================================
            # PHYSICS SIMULATION - Calculate sensor values
            # =====================================================================
            
            # Ice depth varies sinusoidally (oscillates between -2m and 0m)
            ice = -1.0 + (math.sin(t * 0.2) * 1.0)
            
            # Ocean floor depth varies with different frequency (oscillates between -30m and -25m)
            bottom = -27.5 + (math.cos(t * 0.1) * 2.5)
            
            # Noise amplitude depends on operating mode (MANUAL mode is noisier)
            noise_amp = 0.6 if mode == "MANUAL" else 0.1
            
            # Submarine depth oscillates around target, constrained between ice and floor
            sub = max(min(target_depth + (math.sin(t * 0.8) * noise_amp), ice - 0.5), bottom + 0.5)
            
            # Front distance to obstacle: cycles 0-20m
            front = 20.0 - (t % 20)
            
            # =====================================================================
            # ORIENTATION - Calculate 6-DOF orientation (Roll, Pitch, Yaw)
            # =====================================================================
            
            roll = math.sin(t * 1.2) * 0.2
            pitch = math.sin(t * 0.5) * 0.4
            yaw = t * 0.5
            
            # Convert Euler angles to quaternion representation
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            
            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            
            # =====================================================================
            # BUILD MESSAGE PAYLOADS
            # =====================================================================
            
            # Position: oscillates in X/Y plane, Z is depth
            pos = {"x": math.sin(t * 0.1) * 2, "y": math.cos(t * 0.1) * 2, "z": sub}
            
            # Orientation as quaternion
            rot = {"x": qx, "y": qy, "z": qz, "w": qw}
            
            # =====================================================================
            # PUBLISH SENSOR DATA - /sensor/* topics
            # =====================================================================
            
            await server.send_message(
                channels["mode"],
                now_ns,
                json.dumps({"mode": mode}).encode()
            )
            
            await server.send_message(
                channels["ice_depth"],
                now_ns,
                json.dumps({"meters": round(ice, 2)}).encode()
            )
            
            await server.send_message(
                channels["sub_depth"],
                now_ns,
                json.dumps({"meters": round(sub, 2)}).encode()
            )
            
            await server.send_message(
                channels["target_depth"],
                now_ns,
                json.dumps({"meters": round(target_depth, 2)}).encode()
            )
            
            await server.send_message(
                channels["bottom_depth"],
                now_ns,
                json.dumps({"meters": round(bottom, 2)}).encode()
            )
            
            await server.send_message(
                channels["front_dist"],
                now_ns,
                json.dumps({"meters": round(front, 2)}).encode()
            )
            # Simulate battery level (oscillates between 100% and 20%)
            battery_level = 100 - ((t % 100) * 0.8)
            await server.send_message(
                channels["battery_level"],
                now_ns,
                json.dumps({"percentage": round(battery_level, 1)}).encode()
            )
            
            await server.send_message(
                channels["gps"],
                now_ns,
                json.dumps({
                    "latitude": lat + (0.0005 * math.sin(t * 0.05)),
                    "longitude": lon + (0.0005 * math.cos(t * 0.05)),
                    "altitude": sub,
                    "position_covariance_type": 0
                }).encode()
            )
            
            # =====================================================================
            # PUBLISH ACTUATOR DATA - /actuator/thruster_*_rpm topics
            # =====================================================================
            
            # Generate realistic thruster RPMs (6 thrusters with different patterns)
            # Base RPM varies with time, each thruster has slight phase offset
            base_rpm = 1000 + (math.sin(t * 0.5) * 500)
            
            thruster_rpms = [
                base_rpm + (math.sin(t * 0.3 + 0.0) * 200),  # Thruster 1
                base_rpm + (math.sin(t * 0.3 + 1.0) * 200),  # Thruster 2
                base_rpm + (math.sin(t * 0.3 + 2.0) * 200),  # Thruster 3
                base_rpm + (math.sin(t * 0.3 + 3.0) * 200),  # Thruster 4
                base_rpm + (math.sin(t * 0.3 + 4.0) * 200),  # Thruster 5
                base_rpm + (math.sin(t * 0.3 + 5.0) * 200),  # Thruster 6
            ]
            
            for i in range(6):
                await server.send_message(
                    channels[f"thruster_{i+1}_rpm"],
                    now_ns,
                    json.dumps({"rpm": round(thruster_rpms[i], 1)}).encode()
                )
            
            # =====================================================================
            # PUBLISH LOCALIZATION DATA - /sensor/pose, /tf topics
            # =====================================================================
            
            # Publish pose (position + orientation)
            await server.send_message(
                channels["pose"],
                now_ns,
                json.dumps({
                    "position": pos,
                    "orientation": rot
                }).encode()
            )
            
            # Publish coordinate transform (world -> base_link)
            # We keep translation at 0 to keep the sub stationary in the 3D view
            tf_data = {
                "transforms": [{
                    "timestamp": {"sec": int(now), "nsec": int((now % 1) * 1e9)},
                    "parent_frame_id": "world",
                    "child_frame_id": "base_link",
                    "translation": {"x": 0, "y": 0, "z": 0}, # Keeps it stationary at the origin
                    "rotation": rot # This makes it pitch, yaw, and roll
                }]
            }
            await server.send_message(channels["tf"], now_ns, json.dumps(tf_data).encode())
            
            # =====================================================================
            # UPDATED PHYSICS SIMULATION - Realistic Submarine Motion
            # =====================================================================
            
            # 1. VELOCITIES (Body-Fixed)
            # Surge (X): Mostly forward (avg 2.2 m/s) with very slow variation
            vel_x = 2.2 + math.sin(t * 0.2) * 0.4 
            
            # Sway (Y): Slight lateral drift (side-to-side)
            vel_y = math.sin(t * 0.3) * 0.15 
            
            # Heave (Z): Changing vertical movement (diving/surfacing)
            vel_z = math.cos(t * 0.4) * 0.25 

            # 2. ORIENTATION (Smoother Roll and Pitch)
            # Subtle Roll (approx ±3 degrees)
            roll = math.sin(t * 0.4) * 0.05 
            # Subtle Pitch (approx ±6 degrees) - higher frequency for "driving" feel
            pitch = math.sin(t * 0.6) * 0.1  
            # Yaw: Slow, steady rotation
            yaw = t * 0.15 

            # Re-calculate quaternion for these smoother angles
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            
            rot = {
                "w": cr * cp * cy + sr * sp * sy,
                "x": sr * cp * cy - cr * sp * sy,
                "y": cr * sp * cy + sr * cp * sy,
                "z": cr * cp * sy - sr * sp * cy
            }

            # =====================================================================
            # BUILD SCENE UPDATE
            # =====================================================================
            scene_msg = {
                "deletions": [],
                "entities": [
                    {
                        "id": "sub_body",
                        "frame_id": "base_link",
                        "cubes": [{
                            "size": {"x": 1.4, "y": 0.7, "z": 0.5},
                            "pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
                            "color": {"r": 1, "g": 0.8, "b": 0, "a": 1}
                        }]
                    },
                    {
                        "id": "vel_vectors",
                        "frame_id": "base_link",
                        "arrows": [
                            # X-Axis (Surge - RED) - The "Driving" Vector
                            {
                                "shaft_length": abs(vel_x), "shaft_diameter": 0.06, "head_length": 0.15, "head_diameter": 0.12,
                                "pose": {
                                    "position": {"x": 0.7 if vel_x >= 0 else -0.7, "y": 0, "z": 0}, 
                                    "orientation": {"x": 0, "y": 0, "z": 0 if vel_x >= 0 else 1, "w": 1 if vel_x >= 0 else 0}
                                },
                                "color": {"r": 1, "g": 0.1, "b": 0.1, "a": 1}
                            },
                            # Y-Axis (Sway - GREEN) - Subtle drift
                            {
                                "shaft_length": abs(vel_y), "shaft_diameter": 0.04, "head_length": 0.1, "head_diameter": 0.08,
                                "pose": {
                                    "position": {"x": 0, "y": 0.35 if vel_y >= 0 else -0.35, "z": 0}, 
                                    "orientation": {"x": 0, "y": 0, "z": 0.707 if vel_y >= 0 else -0.707, "w": 0.707}
                                },
                                "color": {"r": 0.1, "g": 1, "b": 0.1, "a": 1}
                            },
                            # Z-Axis (Heave - BLUE) - Changing depth
                            {
                                "shaft_length": abs(vel_z), "shaft_diameter": 0.04, "head_length": 0.1, "head_diameter": 0.08,
                                "pose": {
                                    "position": {"x": 0, "y": 0, "z": 0.25 if vel_z >= 0 else -0.25}, 
                                    "orientation": {"x": 0, "y": -0.707 if vel_z >= 0 else 0.707, "z": 0, "w": 0.707}
                                },
                                "color": {"r": 0.1, "g": 0.1, "b": 1, "a": 1}
                            }
                        ],
                        "texts": [
                            {
                                "pose": {"position": {"x": 0, "y": 0, "z": 1.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
                                "text": f"Surge: {vel_x:.2f} m/s\nSway: {vel_y:.2f} m/s\nHeave: {vel_z:.2f} m/s",
                                "font_size": 0.12,
                                "scale_invariant": False, 
                                "color": {"r": 1, "g": 1, "b": 1, "a": 1}
                            }
                        ]
                    }
                ]
            }
            






            await server.send_message(
                channels["scene"],
                now_ns,
                json.dumps(scene_msg).encode()
            )
            
            # =====================================================================
            # PUBLISH DIAGNOSTICS - /diagnostics topic
            # =====================================================================
            
            # Calculate gap between submarine and ice ceiling
            ice_gap = abs(sub - ice)
            
            # Determine health levels based on sensor values
            ice_level = 0 if ice_gap > 1.5 else (1 if ice_gap > 0.5 else 2)
            obstacle_level = 0 if front > 5.0 else 1
            
            diag_msg = {
                "header": {
                    "stamp": {"sec": int(now), "nsec": int((now % 1) * 1e9)},
                    "frame_id": "base_link"
                },
                "status": [
                    # Environment altitude check
                    {
                        "level": ice_level,
                        "name": "Env:Altitudes",
                        "message": "OK",
                        "hardware_id": "sonar_1",
                        "values": [
                            {"key": "Ice", "value": f"{ice:.2f}m"},
                            {"key": "Bottom", "value": f"{bottom:.2f}m"}
                        ]
                    },
                    # Obstacle detection
                    {
                        "level": obstacle_level,
                        "name": "Env:Obstacles",
                        "message": "Clear",
                        "hardware_id": "sonar_2",
                        "values": [
                            {"key": "Front Dist", "value": f"{front:.2f}m"}
                        ]
                    },
                    # Control system status
                    {
                        "level": 0,
                        "name": "Sys:Control",
                        "message": mode,
                        "hardware_id": "pid_1",
                        "values": [
                            {"key": "Target", "value": f"{target_depth:.2f}m"},
                            {"key": "Actual", "value": f"{sub:.2f}m"}
                        ]
                    }
                ]
            }
            await server.send_message(
                channels["diagnostics"],
                now_ns,
                json.dumps(diag_msg).encode()
            )
            
            # Publish at 10 Hz (0.1 second interval)
            await asyncio.sleep(0.1)





if __name__ == "__main__":
    # Run the async main function
    asyncio.run(main())