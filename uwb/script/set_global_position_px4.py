from pymavlink import mavutil

master = mavutil.mavlink_connection('127.0.0.1:14550')
master.wait_heartbeat()
print("Heartbeat Check",master.target_system,master.target_component)

lat = int(47.397742 * 1e7)
lon = int(8.545594 * 1e7)
alt = int(500*1000)

master.mav.set_gps_global_origin_send(
        master.target_system,
        lat,
        lon,
        alt
        )
master.mav.set_home_position_send(
        master.target_system,
        lat,
        lon,
        alt,
        0,0,0,
        [1,0,0,0],
        0,0,0
        )
print("Finish")
