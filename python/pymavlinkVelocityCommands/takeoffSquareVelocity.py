#fly in a square

from pymavlink import mavutil

import time

TAKEOFF_ALT_SCALER = 0.9

class VehicleState(object):
	preFlight = "PREFLIGHT"
	takeoff = "TAKEOFF"
	auto = "AUTO"
	manual = "MANUAL"
	landing = "LANDING"
	landed = "LANDED"

state = VehicleState.preFlight
def makewp(self, seq):
        return mavutil.mavlink.MAVLink_mission_item_int_message(
            1, # wp.target_system,
            1, # wp.target_component,
            0, # wp.seq,
            0, # wp.frame,
            0, # wp.command,
            0, # wp.current,
            1, # wp.autocontinue,
            0, # wp.param1,
            0, # wp.param2,
            0, # wp.param3,
            0, # wp.param4,
            0, # int(wp.x*1.0e7),
            0, # int(wp.y*1.0e7),
            seq, # wp.z
        )
def sendMission (mav_conn):
    mav_conn.waypoint_clear_all_send()
    time.sleep(2)  # Wait for the clear to complete

    # Define the waypoint
    
    # waypoint = mavutil.mavlink.MAVLink_mission_item_message(
    #     mav_conn.target_system,
    #     mav_conn.target_component,
    #     0,  # Sequence number
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Global frame, relative altitude
    #     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # The command for navigating to a waypoint
    #     1,  # Current waypoint - set to true to make it active
    #     1,  # Autocontinue - set to true
    #     0, 0, 0, 0,  # Params 1-4 are not used
    #     lat, lon, 0  # Latitude, Longitude, Altitude
    # )
    home_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            mav_conn.target_system, # wp.target_system,
            mav_conn.target_component, # wp.target_component,
            0, # wp.seq,
            0, # wp.frame,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME, # wp.command,
            0, # wp.current,
            1, # wp.autocontinue,
            0, # wp.param1,
            0, # wp.param2,
            0, # wp.param3,
            0, # wp.param4,
            0, # int(wp.x*1.0e7),
            0, # int(wp.y*1.0e7),
            0, # wp.z
        )
    
    takeoff_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            mav_conn.target_system, # wp.target_system,
            mav_conn.target_component, # wp.target_component,
            1, # wp.seq,
            0, # wp.frame,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # wp.command,
            0, # wp.current,
            1, # wp.autocontinue,
            0, # wp.param1,
            0, # wp.param2,
            0, # wp.param3,
            0, # wp.param4,
            0, # int(wp.x*1.0e7),
            0, # int(wp.y*1.0e7),
            0.5, # wp.z
        )

    loiter_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            mav_conn.target_system, # wp.target_system,
            mav_conn.target_component, # wp.target_component,
            3, # wp.seq,
            0, # wp.frame,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, # wp.command,
            0, # wp.current,
            0, # wp.autocontinue,
            0, # wp.param1,
            0, # wp.param2,
            0, # wp.param3,
            0, # wp.param4,
            0, # int(wp.x*1.0e7),
            0, # int(wp.y*1.0e7),
            0, # wp.z
        )
    
    

    # Send waypoint count (1 in this case)
    mav_conn.waypoint_count_send(3)

    # Wait for the vehicle to request the waypoint
    request = mav_conn.recv_match(type='MISSION_REQUEST', blocking=True)
    print("Vehicle requested waypoint", request.seq)

    # Send the defined waypoint
    mav_conn.mav.send(home_waypoint)
    print("Sending waypoint: home_waypoint")

    # Wait for the vehicle to request the waypoint
    request = mav_conn.recv_match(type='MISSION_REQUEST', blocking=True)
    print("Vehicle requested waypoint", request.seq)

    # Send the defined waypoint
    mav_conn.mav.send(takeoff_waypoint)
    print("Sending waypoint: takeoff_waypoint")
    # Wait for the vehicle to request the waypoint
    request = mav_conn.recv_match(type='MISSION_REQUEST', blocking=True)
    print("Vehicle requested waypoint", request.seq)

    # Send the defined waypoint
    mav_conn.mav.send(loiter_waypoint)
    print("Sending waypoint: loiter_waypoint")


    # Wait for mission acknowledgment
    ack = mav_conn.recv_match(type='MISSION_ACK', blocking=True)
    if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission accepted by vehicle.")
    else:
        print("Mission upload failed with error:", ack.type)

def start_mission(mav_connection):
     mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_MISSION_START,
                                0,          # confirmation
						0,          # param 1, use absolute location
						0, 0, 0,    # param 2-4, not used
						0,        # param 5
						0,        # param 6 
						0)        # param 7

      
def set_home(mav_connection, lng = 0, lat = 1, alt = 0):
		
        mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                                0,          # confirmation
						0,          # param 1, use absolute location
						0, 0, 0,    # param 2-4, not used
						lat,        # param 5, latitude
						lng,        # param 6, longitude 
						alt)        # param 7, altitude
        ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"set_home ACK:  {ack_msg}")

def set_velocity_body(mav_connection, vx, vy, vz, yaw=0):
    mav_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b100111000111),
                        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0))
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"set_velocity_body ACK:  {ack_msg}")

def set_position_body_offset(mav_connection, x, y, z, yaw=0):
    mav_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b100111111000),
                        x, y, z, 0, 0, 0, 0, 0, 0, 0, 0))
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"set_position_body_offset ACK:  {ack_msg}")

def set_guided(mav_connection):
     # Change mode to guided (Ardupilot) or takeoff (PX4)
    mode_id = mav_connection.mode_mapping()["GUIDED"]
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

def set_auto(mav_connection):
     # Change mode to guided (Ardupilot) or takeoff (PX4)
    mode_id = mav_connection.mode_mapping()["AUTO"]
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

def arm(mav_connection):
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

def disarm(mav_connection):
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

def estop(mav_connection):
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Force disarm (ESTOP) ACK:  {arm_msg}")

def takeoff(mav_connection, targetHeight, wait_ready = True):
    state = VehicleState.takeoff
    msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    startHeightGlobal = msg.alt / 1000
    print(f"startHeightGlobal: {startHeightGlobal}")
    print(f"GLOBAL_POSITION_INT: {msg}")
    targetHeightGlobal = startHeightGlobal + targetHeight
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, targetHeight)
		
		
    print("Vehicle is taking off!")
    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    while wait_ready:
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        currentAlt = msg.alt / 1000
        print(f"GLOBAL_POSITION_INT: {msg}")

        if state != VehicleState.takeoff:
            print("Err: Takeoff terminated unexpectedly with state", state)
            return False
        print(" Current relativealtitude: ", currentAlt - startHeightGlobal, "\n"); 
			#Break and return from function just below target altitude.        
        if currentAlt - startHeightGlobal >= targetHeight * TAKEOFF_ALT_SCALER: 
            print("Reached target altitude")
            state = VehicleState.auto
            break
			
    return True

def set_rc_channel_pwm(mav_connection, channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    mav_connection.mav.rc_channels_override_send(
        mav_connection.target_system,                # target_system
        mav_connection.target_component,             # target_component
        *rc_channel_values)

def hover(mav_connection, duration):
	state = VehicleState.auto;
	
	# Set RC3(throttle) to the hover level
	#print("sending command override")
     # create the LOITER_UNLIM command
	print("command override complete")
	mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        0, 0, 0, 0, 0, 0, 0, 0)
	ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
	print(f"Unlim Loiter ACK:  {ack_msg}")
	print(f"waiting for {duration}")
	time.sleep(duration)
	print("releasing override")
	#set_rc_channel_pwm(mav_connection, 3, pwm=0)
	return True

def land(mav_connection):
    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mode_id = mav_connection.mode_mapping()["LAND"]
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode (LAND) ACK:  {ack_msg}")

def main():
    
    
    mav_connection = mavutil.mavlink_connection('tcp:localhost:14540')
    print("waiting for heartbeat")
    mav_connection.wait_heartbeat()

    print("sending mission: ")
    print("main: set_guided")
    set_guided(mav_connection)
    print("main: arm")
    arm(mav_connection)
    print("main: takeoff")
    takeoff(mav_connection, 0.5, wait_ready=False)
    time.sleep(3)
    set_velocity_body(mav_connection, 0.3, 0, 0)
    time.sleep(2)
    set_velocity_body(mav_connection, 0, 0.3, 0)
    time.sleep(2)
    set_velocity_body(mav_connection, -0.3, 0, 0)
    time.sleep(2)
    set_velocity_body(mav_connection, 0, -0.3, 0)
    time.sleep(2)
    print("main: land")
    land(mav_connection)
    time.sleep(3)
    disarm(mav_connection)

if __name__ == "__main__":
    main()
