import argparse
import csv
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import WrenchStamped

def export_rosbag_to_csv(bag_file, output_csv):
    # Initialize the ROS 2 node
    rclpy.init()

    # Set up the SequentialReader to read the rosbag2 file
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_file, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Retrieve and filter topics
    topic_types = reader.get_all_topics_and_types()
    topic_type_dict = {topic.name: topic.type for topic in topic_types}

    if "/force_sensor_node/data" not in topic_type_dict:
        print("Error: /force_sensor_node/data not found in the bag file.")
        return

    if topic_type_dict["/force_sensor_node/data"] != "geometry_msgs/msg/WrenchStamped":
        print("Error: /force_sensor_node/data is not of type geometry_msgs/msg/WrenchStamped.")
        return

    # Prepare CSV file for writing
    with open(output_csv, mode="w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        # Write header
        csv_writer.writerow(["timestamp", "force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"])

        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            if topic == "/force_sensor_node/data":
                # Deserialize the message
                msg = deserialize_message(data, WrenchStamped)
                # Extract data
                csv_writer.writerow([
                    timestamp,  # Nanoseconds since epoch
                    msg.wrench.force.x,
                    msg.wrench.force.y,
                    msg.wrench.force.z,
                    msg.wrench.torque.x,
                    msg.wrench.torque.y,
                    msg.wrench.torque.z
                ])

    print(f"Data successfully exported to {output_csv}")

    # Shutdown the ROS 2 context
    rclpy.shutdown()

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Convert ROS2 bag file to CSV for WrenchStamped data.")
    parser.add_argument("bag_file", help="Path to the ROS2 bag file")
    parser.add_argument("output_csv", help="Path to the output CSV file")
    args = parser.parse_args()

    # Run the export function
    export_rosbag_to_csv(args.bag_file, args.output_csv)
