#This script deserializes the rosbag recording and outputs the logs to text files
#BagFileParser() code found here: https://github.com/ros2/rosbag2/issues/473

import sqlite3
import sys
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        # create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of,
                           name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of,
                         name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(
            type_of) for id_of, name_of, type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]

#arg1 is input rosbag.db3 file
#arg2 is output directory 
if __name__ == "__main__":

    #bag_file = input("Bag file location: ")
    bag_file = str(sys.argv[1])
    parser = BagFileParser(bag_file)

    outputDir = str(sys.argv[2])

    #topic parameter should be in the form "/topic" to work with parser
    def LogTopic(topic):
        """Parses topic and logs output to a text file"""
        
        #create log file to output deserialized data to. Topic name may throw error if it contains any forward slashes. 
        log = open(outputDir + topic[1:] + "_log.txt", "w")

        print("\nRecording messages from " + topic + "\n")
         
        messages = parser.get_messages(topic)

        log.write("\nMessages recorded from " + topic + "\n")

        for msg in messages:

            line = "Timestamp: {} Message: {}\n\n".format(
                msg[0], msg[1])

            log.write(line)

        print("Finished recording topic " + topic + "\n")
        log.close()

    #Simply call LogTopic("/topic") for all topics recorded by rosbag
    LogTopic("/odom")
    LogTopic("/rosout")
    LogTopic("/tf")
