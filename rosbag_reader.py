import os
import json
import numpy

from mcap.reader import make_reader
from rosbags.highlevel.anyreader import deserialize_cdr

MCAP_FILENAME = os.path.expanduser("~/data/rssi_measurement_set1_mcap/sad03_mission1.mcap")

def get_dict_from_mcap(filename):
    with open(filename, "rb") as f:
        reader = make_reader(f)
        logs = {c.topic: {'timestamps': [], 'data': []} for c in reader.get_summary().channels.values()}
        for schema, channel, message in reader.iter_messages():
            logs[channel.topic]['timestamps'].append(message.log_time)
            try:
                logs[channel.topic]['data'].append(json.loads(deserialize_cdr(message.data, schema.name).data))
                if (len(logs[channel.topic]['data']) == 1):
                    print(f"SUCCESS: '{channel.topic}' was deserialized :)")
            except:
                if (len(logs[channel.topic]['data']) == 0):
                    print(f"FAILED: '{channel.topic}' could not be deserialized :(")
                logs[channel.topic]['data'].append(message.data)

    return logs
        
# --- Sync RSSI values with receiver positions -

RSSI_TOPIC = '/sad03/mesh_visual'

def sync_logs(logs: dict, to_topic: str, from_topic: str):
    """
    param from_topic: topic to be used as the reference
    param to_topic: topic to be synced

    For e.g., Let's say from_topic is "RSSI" and to_topic is "GPS position of the receiver"
    if 400 RSSI values are present in the logs, then it collects the corresponding 400 positions from the GPS logs, and puts them inside the dict, logs[from_topic]
    """
    logs[from_topic][to_topic] = []
  
    to_index = 0
    from_index = 0
    while to_index < len(logs[to_topic]['data']) and from_index < len(logs[from_topic]['data']):
        if logs[to_topic]['timestamps'][to_index] > logs[from_topic]['timestamps'][from_index]:
            logs[from_topic][to_topic].append(logs[to_topic]['data'][to_index])
            from_index += 1
        to_index += 1
    
    if from_index < len(logs[from_topic]['data']):
        raise NotImplementedError("We have not handled this case...")

    return logs

_ = get_dict_from_mcap(MCAP_FILENAME)