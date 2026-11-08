# Diagnostic timing of an accepted MAVLink MANUAL_CONTROL message at the receiver.
# The sequence number advances for all MAVLink messages from a peer, so a gap
# between two values does not by itself prove lost manual-control packets.

uint64 timestamp             # local time after MAVLink frame decoding (microseconds)
uint32 arrival_gap_us        # time since previous accepted manual input on this receiver; 0 for first/peer change
uint8 mavlink_instance       # PX4 MAVLink instance that received the message
uint8 system_id              # sender MAVLink system ID
uint8 component_id           # sender MAVLink component ID
uint8 sequence               # sender MAVLink frame sequence (shared by all message types)
