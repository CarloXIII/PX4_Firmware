// MESSAGE XSENS_GLOB_POS PACKING

#define MAVLINK_MSG_ID_XSENS_GLOB_POS 206

typedef struct __mavlink_xsens_glob_pos_t
{
 uint64_t timestamp; ///< time of this estimate, in microseconds since system start
 int32_t lat; ///< Latitude in 1E7 degrees
 int32_t lon; ///< Longitude in 1E7 degrees	
 float alt; ///< Altitude in meters
 float rel_alt; ///< Altitude above home position in meters,
 float vel_x; ///< Ground X velocity, m/s in NED
 float vel_y; ///< Ground Y velocity, m/s in NED
 float vel_z; ///< Ground Z velocity, m/s	in NED
 float yaw; ///< Compass heading in radians -PI..+PI
} mavlink_xsens_glob_pos_t;

#define MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN 40
#define MAVLINK_MSG_ID_206_LEN 40

#define MAVLINK_MSG_ID_XSENS_GLOB_POS_CRC 131
#define MAVLINK_MSG_ID_206_CRC 131



#define MAVLINK_MESSAGE_INFO_XSENS_GLOB_POS { \
	"XSENS_GLOB_POS", \
	9, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_xsens_glob_pos_t, timestamp) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_xsens_glob_pos_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_xsens_glob_pos_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_xsens_glob_pos_t, alt) }, \
         { "rel_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_xsens_glob_pos_t, rel_alt) }, \
         { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_xsens_glob_pos_t, vel_x) }, \
         { "vel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_xsens_glob_pos_t, vel_y) }, \
         { "vel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_xsens_glob_pos_t, vel_z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_xsens_glob_pos_t, yaw) }, \
         } \
}


/**
 * @brief Pack a xsens_glob_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp time of this estimate, in microseconds since system start
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees	
 * @param alt Altitude in meters
 * @param rel_alt Altitude above home position in meters,
 * @param vel_x Ground X velocity, m/s in NED
 * @param vel_y Ground Y velocity, m/s in NED
 * @param vel_z Ground Z velocity, m/s	in NED
 * @param yaw Compass heading in radians -PI..+PI
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_glob_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, int32_t lat, int32_t lon, float alt, float rel_alt, float vel_x, float vel_y, float vel_z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, rel_alt);
	_mav_put_float(buf, 24, vel_x);
	_mav_put_float(buf, 28, vel_y);
	_mav_put_float(buf, 32, vel_z);
	_mav_put_float(buf, 36, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#else
	mavlink_xsens_glob_pos_t packet;
	packet.timestamp = timestamp;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.rel_alt = rel_alt;
	packet.vel_x = vel_x;
	packet.vel_y = vel_y;
	packet.vel_z = vel_z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_GLOB_POS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN, MAVLINK_MSG_ID_XSENS_GLOB_POS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#endif
}

/**
 * @brief Pack a xsens_glob_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp time of this estimate, in microseconds since system start
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees	
 * @param alt Altitude in meters
 * @param rel_alt Altitude above home position in meters,
 * @param vel_x Ground X velocity, m/s in NED
 * @param vel_y Ground Y velocity, m/s in NED
 * @param vel_z Ground Z velocity, m/s	in NED
 * @param yaw Compass heading in radians -PI..+PI
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_glob_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,int32_t lat,int32_t lon,float alt,float rel_alt,float vel_x,float vel_y,float vel_z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, rel_alt);
	_mav_put_float(buf, 24, vel_x);
	_mav_put_float(buf, 28, vel_y);
	_mav_put_float(buf, 32, vel_z);
	_mav_put_float(buf, 36, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#else
	mavlink_xsens_glob_pos_t packet;
	packet.timestamp = timestamp;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.rel_alt = rel_alt;
	packet.vel_x = vel_x;
	packet.vel_y = vel_y;
	packet.vel_z = vel_z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_GLOB_POS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN, MAVLINK_MSG_ID_XSENS_GLOB_POS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#endif
}

/**
 * @brief Encode a xsens_glob_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param xsens_glob_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_glob_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_xsens_glob_pos_t* xsens_glob_pos)
{
	return mavlink_msg_xsens_glob_pos_pack(system_id, component_id, msg, xsens_glob_pos->timestamp, xsens_glob_pos->lat, xsens_glob_pos->lon, xsens_glob_pos->alt, xsens_glob_pos->rel_alt, xsens_glob_pos->vel_x, xsens_glob_pos->vel_y, xsens_glob_pos->vel_z, xsens_glob_pos->yaw);
}

/**
 * @brief Encode a xsens_glob_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xsens_glob_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_glob_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_xsens_glob_pos_t* xsens_glob_pos)
{
	return mavlink_msg_xsens_glob_pos_pack_chan(system_id, component_id, chan, msg, xsens_glob_pos->timestamp, xsens_glob_pos->lat, xsens_glob_pos->lon, xsens_glob_pos->alt, xsens_glob_pos->rel_alt, xsens_glob_pos->vel_x, xsens_glob_pos->vel_y, xsens_glob_pos->vel_z, xsens_glob_pos->yaw);
}

/**
 * @brief Send a xsens_glob_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp time of this estimate, in microseconds since system start
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees	
 * @param alt Altitude in meters
 * @param rel_alt Altitude above home position in meters,
 * @param vel_x Ground X velocity, m/s in NED
 * @param vel_y Ground Y velocity, m/s in NED
 * @param vel_z Ground Z velocity, m/s	in NED
 * @param yaw Compass heading in radians -PI..+PI
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_xsens_glob_pos_send(mavlink_channel_t chan, uint64_t timestamp, int32_t lat, int32_t lon, float alt, float rel_alt, float vel_x, float vel_y, float vel_z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, rel_alt);
	_mav_put_float(buf, 24, vel_x);
	_mav_put_float(buf, 28, vel_y);
	_mav_put_float(buf, 32, vel_z);
	_mav_put_float(buf, 36, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GLOB_POS, buf, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN, MAVLINK_MSG_ID_XSENS_GLOB_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GLOB_POS, buf, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#endif
#else
	mavlink_xsens_glob_pos_t packet;
	packet.timestamp = timestamp;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.rel_alt = rel_alt;
	packet.vel_x = vel_x;
	packet.vel_y = vel_y;
	packet.vel_z = vel_z;
	packet.yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GLOB_POS, (const char *)&packet, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN, MAVLINK_MSG_ID_XSENS_GLOB_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GLOB_POS, (const char *)&packet, MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#endif
#endif
}

#endif

// MESSAGE XSENS_GLOB_POS UNPACKING


/**
 * @brief Get field timestamp from xsens_glob_pos message
 *
 * @return time of this estimate, in microseconds since system start
 */
static inline uint64_t mavlink_msg_xsens_glob_pos_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from xsens_glob_pos message
 *
 * @return Latitude in 1E7 degrees
 */
static inline int32_t mavlink_msg_xsens_glob_pos_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from xsens_glob_pos message
 *
 * @return Longitude in 1E7 degrees	
 */
static inline int32_t mavlink_msg_xsens_glob_pos_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from xsens_glob_pos message
 *
 * @return Altitude in meters
 */
static inline float mavlink_msg_xsens_glob_pos_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rel_alt from xsens_glob_pos message
 *
 * @return Altitude above home position in meters,
 */
static inline float mavlink_msg_xsens_glob_pos_get_rel_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vel_x from xsens_glob_pos message
 *
 * @return Ground X velocity, m/s in NED
 */
static inline float mavlink_msg_xsens_glob_pos_get_vel_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vel_y from xsens_glob_pos message
 *
 * @return Ground Y velocity, m/s in NED
 */
static inline float mavlink_msg_xsens_glob_pos_get_vel_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vel_z from xsens_glob_pos message
 *
 * @return Ground Z velocity, m/s	in NED
 */
static inline float mavlink_msg_xsens_glob_pos_get_vel_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field yaw from xsens_glob_pos message
 *
 * @return Compass heading in radians -PI..+PI
 */
static inline float mavlink_msg_xsens_glob_pos_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a xsens_glob_pos message into a struct
 *
 * @param msg The message to decode
 * @param xsens_glob_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_xsens_glob_pos_decode(const mavlink_message_t* msg, mavlink_xsens_glob_pos_t* xsens_glob_pos)
{
#if MAVLINK_NEED_BYTE_SWAP
	xsens_glob_pos->timestamp = mavlink_msg_xsens_glob_pos_get_timestamp(msg);
	xsens_glob_pos->lat = mavlink_msg_xsens_glob_pos_get_lat(msg);
	xsens_glob_pos->lon = mavlink_msg_xsens_glob_pos_get_lon(msg);
	xsens_glob_pos->alt = mavlink_msg_xsens_glob_pos_get_alt(msg);
	xsens_glob_pos->rel_alt = mavlink_msg_xsens_glob_pos_get_rel_alt(msg);
	xsens_glob_pos->vel_x = mavlink_msg_xsens_glob_pos_get_vel_x(msg);
	xsens_glob_pos->vel_y = mavlink_msg_xsens_glob_pos_get_vel_y(msg);
	xsens_glob_pos->vel_z = mavlink_msg_xsens_glob_pos_get_vel_z(msg);
	xsens_glob_pos->yaw = mavlink_msg_xsens_glob_pos_get_yaw(msg);
#else
	memcpy(xsens_glob_pos, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_XSENS_GLOB_POS_LEN);
#endif
}
