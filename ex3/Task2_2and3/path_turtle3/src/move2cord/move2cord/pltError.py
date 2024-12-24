import rosbag
import pandas as pd
from std_msgs.msg import Float64
import rospy

def extract_distance_errors(bag_file):
    distance_errors = []
    timestamps = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/distance_error']):
            distance_errors.append(msg.data)
            timestamps.append(t.to_sec())  # Convert ROS time to seconds

    return timestamps, distance_errors

if __name__ == '__main__':
    rospy.init_node('bag_data_extractor', anonymous=True)
    bag_file = 'performance_data.bag'  # Adjust the path if necessary

    timestamps, distance_errors = extract_distance_errors(bag_file)

    # Save to CSV for further analysis
    df = pd.DataFrame({'timestamp': timestamps, 'distance_error': distance_errors})
    df.to_csv('distance_errors.csv', index=False)
    rospy.loginfo("Distance errors extracted to distance_errors.csv")
