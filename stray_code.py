raise NotImplementedError("Please do not run this file :')")

with open(os.path.expanduser("~/local/data/rssi_measurement_set1.json"), "rb") as f:
    data = json.load(f)

topics  =  ['/fmu/out/VehicleGpsPosition', '/sad03/drone/global_position', '/sad03/drone/home_position', '/sad03/drone/local_odom', '/sad03/drone/local_position', '/sad03/mesh_visual']

# --- Populate fields of data -

with open(MCAP_FILENAME, "rb") as f:
    reader = make_reader(f)
    for schema, channel, message in reader.iter_messages("/sad03/mesh_visual"):
        data["rssi"].append(
            json.loads(deserialize_cdr(message.data, schema.name).data)
            )
        data["rssi_timestamps"].append(message.log_time)

        # --- Get other data:
        channels = reader.get_summary.channels()



    data["rssi_positions"] = []
    position_index = 0
    rssi_index = 0
    with open(MCAP_FILENAME, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages('/sad03/drone/global_position'):
            if data["rssi_timestamps"][rssi_index] < message.log_time:
                
                # TODO: Get data xyz from the mcap dataset here...

                data["rssi_positions"].append((data["x"][position_index], 
                                            data["y"][position_index], 
                                            data["z"][position_index]))
                rssi_index += 1
                if rssi_index >= len(data["rssi"]):
                    break
            position_index += 1
    

    # --- Get TX-RX distances from RSSI -
    data["distances"] = []
    origin_xyz = numpy.array([[data['x'][0]], [data['y'][0]], [data['z'][0]]])
    for rssi_xyz in zip(data["rssi"], data["rssi_positions"]):
        receiver = numpy.array([rssi_xyz[1][0], rssi_xyz[1][1], rssi_xyz[1][2]])
        distance = numpy.linalg.norm(receiver - origin_xyz)
        data["distances"].append(distance)

    for topic in topics:
