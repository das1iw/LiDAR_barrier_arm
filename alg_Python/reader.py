from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

def read_points(bag_path, topic="/front/rslidar_points"):
    reader = SequentialReader()
    storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter = ConverterOptions('', '')
    reader.open(storage, converter)

    frames = []

    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, PointCloud2)
            pts = np.array(list(pc2.read_points(msg, skip_nans=True)))
            frames.append(pts)

    return frames