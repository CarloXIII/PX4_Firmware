// MESSAGE XSENS_ATTITUDE PACKING

#define MAVLINK_MSG_ID_XSENS_ATTITUDE 205

typedef struct __mavlink_xsens_attitude_t
{
 uint64_t timestamp; ///< in microseconds since system start
 float roll; ///< Roll angle (rad, Tait-Bryan, NED
 float pitch; ///< Pitch angle (rad, Tait-Bryan, NED)	
 float yaw; ///< Yaw angle (rad, Tait-Bryan, NED)
 float rollspeed; ///< Roll angular speed (rad/s, Tait-Bryan, NED)
 float pitchspeed; ///< Pitch angular speed (rad/s, Tait-Bryan, NED)
 float yawspeed; ///< Yaw angular speed (rad/s, Tait-Bryan, NED)
 float rollacc; ///< Roll angular accelration (rad/s, Tait-Bryan, NED)
 float pitchacc; ///< Pitch angular acceleration (rad/s, Tait-Bryan, NED)
 float yawacc; ///< Yaw angular acceleration (rad/s, Tait-Bryan, NED)
} mavlink_xsens_attitude_t;

#define MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN 44
#define MAVLINK_MSG_ID_205_LEN 44

#define MAVLINK_MSG_ID_XSENS_ATTITUDE_CRC 121
#define MAVLINK_MSG_ID_205_CRC 121



#define MAVLINK_MESSAGE_INFO_XSENS_ATTITUDE { \
	"XSENS_ATTITUDE", \
	10, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_xsens_attitude_t, timestamp) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_xsens_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_xsens_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_xsens_attitude_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_xsens_attitude_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_xsens_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_xsens_attitude_t, yawspeed) }, \
         { "rollacc", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_xsens_attitude_t, rollacc) }, \
         { "pitchacc", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_xsens_attitude_t, pitchacc) }, \
         { "yawacc", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_xsens_attitude_t, yawacc) }, \
         } \
}


/**
 * @brief Pack a xsens_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp in microseconds since system start
 * @param roll Roll angle (rad, Tait-Bryan, NED
 * @param pitch Pitch angle (rad, Tait-Bryan, NED)	
 * @param yaw Yaw angle (rad, Tait-Bryan, NED)
 * @param rollspeed Roll angular speed (rad/s, Tait-Bryan, NED)
 * @param pitchspeed Pitch angular speed (rad/s, Tait-Bryan, NED)
 * @param yawspeed Yaw angular speed (rad/s, Tait-Bryan, NED)
 * @param rollacc Roll angular accelration (rad/s, Tait-Bryan, NED)
 * @param pitchacc Pitch angular acceleration (rad/s, Tait-Bryan, NED)
 * @param yawacc Yaw angular acceleration (rad/s, Tait-Bryan, NED)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, float rollacc, float pitchacc, float yawacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);
	_mav_put_float(buf, 32, rollacc);
	_mav_put_float(buf, 36, pitchacc);
	_mav_put_float(buf, 40, yawacc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#else
	mavlink_xsens_attitude_t packet;
	packet.timestamp = timestamp;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.rollacc = rollacc;
	packet.pitchacc = pitchacc;
	packet.yawacc = yawacc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN, MAVLINK_MSG_ID_XSENS_ATTITUDE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a xsens_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp in microseconds since system start
 * @param roll Roll angle (rad, Tait-Bryan, NED
 * @param pitch Pitch angle (rad, Tait-Bryan, NED)	
 * @param yaw Yaw angle (rad, Tait-Bryan, NED)
 * @param rollspeed Roll angular speed (rad/s, Tait-Bryan, NED)
 * @param pitchspeed Pitch angular speed (rad/s, Tait-Bryan, NED)
 * @param yawspeed Yaw angular speed (rad/s, Tait-Bryan, NED)
 * @param rollacc Roll angular accelration (rad/s, Tait-Bryan, NED)
 * @param pitchacc Pitch angular acceleration (rad/s, Tait-Bryan, NED)
 * @param yawacc Yaw angular acceleration (rad/s, Tait-Bryan, NED)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed,float rollacc,float pitchacc,float yawacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);
	_mav_put_float(buf, 32, rollacc);
	_mav_put_float(buf, 36, pitchacc);
	_mav_put_float(buf, 40, yawacc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#else
	mavlink_xsens_attitude_t packet;
	packet.timestamp = timestamp;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.rollacc = rollacc;
	packet.pitchacc = pitchacc;
	packet.yawacc = yawacc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN, MAVLINK_MSG_ID_XSENS_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#endif
}

/**
 * @brief Encode a xsens_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param xsens_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_xsens_attitude_t* xsens_attitude)
{
	return mavlink_msg_xsens_attitude_pack(system_id, component_id, msg, xsens_attitude->timestamp, xsens_attitude->roll, xsens_attitude->pitch, xsens_attitude->yaw, xsens_attitude->rollspeed, xsens_attitude->pitchspeed, xsens_attitude->yawspeed, xsens_attitude->rollacc, xsens_attitude->pitchacc, xsens_attitude->yawacc);
}

/**
 * @brief Encode a xsens_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xsens_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_xsens_attitude_t* xsens_attitude)
{
	return mavlink_msg_xsens_attitude_pack_chan(system_id, component_id, chan, msg, xsens_attitude->timestamp, xsens_attitude->roll, xsens_attitude->pitch, xsens_attitude->yaw, xsens_attitude->rollspeed, xsens_attitude->pitchspeed, xsens_attitude->yawspeed, xsens_attitude->rollacc, xsens_attitude->pitchacc, xsens_attitude->yawacc);
}

/**
 * @brief Send a xsens_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp in microseconds since system start
 * @param roll Roll angle (rad, Tait-Bryan, NED
 * @param pitch Pitch angle (rad, Tait-Bryan, NED)	
 * @param yaw Yaw angle (rad, Tait-Bryan, NED)
 * @param rollspeed Roll angular speed (rad/s, Tait-Bryan, NED)
 * @param pitchspeed Pitch angular speed (rad/s, Tait-Bryan, NED)
 * @param yawspeed Yaw angular speed (rad/s, Tait-Bryan, NED)
 * @param rollacc Roll angular accelration (rad/s, Tait-Bryan, NED)
 * @param pitchacc Pitch angular acceleration (rad/s, Tait-Bryan, NED)
 * @param yawacc Yaw angular acceleration (rad/s, Tait-Bryan, NED)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_xsens_attitude_send(mavlink_channel_t chan, uint64_t timestamp, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, float rollacc, float pitchacc, float yawacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, pitch);
	_mav_put_float(buf, 16, yaw);
	_mav_put_float(buf, 20, rollspeed);
	_mav_put_float(buf, 24, pitchspeed);
	_mav_put_float(buf, 28, yawspeed);
	_mav_put_float(buf, 32, rollacc);
	_mav_put_float(buf, 36, pitchacc);
	_mav_put_float(buf, 40, yawacc);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_ATTITUDE, buf, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN, MAVLINK_MSG_ID_XSENS_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_ATTITUDE, buf, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#endif
#else
	mavlink_xsens_attitude_t packet;
	packet.timestamp = timestamp;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.rollacc = rollacc;
	packet.pitchacc = pitchacc;
	packet.yawacc = yawacc;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN, MAVLINK_MSG_ID_XSENS_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#endif
#endif
}

#endif

// MESSAGE XSENS_ATTITUDE UNPACKING


/**
 * @brief Get field timestamp from xsens_attitude message
 *
 * @return in microseconds since system start
 */
static inline uint64_t mavlink_msg_xsens_attitude_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field roll from xsens_attitude message
 *
 * @return Roll angle (rad, Tait-Bryan, NED
 */
static inline float mavlink_msg_xsens_attitude_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from xsens_attitude message
 *
 * @return Pitch angle (rad, Tait-Bryan, NED)	
 */
static inline float mavlink_msg_xsens_attitude_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw from xsens_attitude message
 *
 * @return Yaw angle (rad, Tait-Bryan, NED)
 */
static inline float mavlink_msg_xsens_attitude_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rollspeed from xsens_attitude message
 *
 * @return Roll angular speed (rad/s, Tait-Bryan, NED)
 */
static inline float mavlink_msg_xsens_attitude_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitchspeed from xsens_attitude message
 *
 * @return Pitch angular speed (rad/s, Tait-Bryan, NED)
 */
static inline float mavlink_msg_xsens_attitude_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yawspeed from xsens_attitude message
 *
 * @return Yaw angular speed (rad/s, Tait-Bryan, NED)
 */
static inline float mavlink_msg_xsens_attitude_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field rollacc from xsens_attitude message
 *
 * @return Roll angular accelration (rad/s, Tait-Bryan, NED)
 */
static inline float mavlink_msg_xsens_attitude_get_rollacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field pitchacc from xsens_attitude message
 *
 * @return Pitch angular acceleration (rad/s, Tait-Bryan, NED)
 */
static inline float mavlink_msg_xsens_attitude_get_pitchacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field yawacc from xsens_attitude message
 *
 * @return Yaw angular acceleration (rad/s, Tait-Bryan, NED)
 */
static inline float mavlink_msg_xsens_attitude_get_yawacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a xsens_attitude message into a struct
 *
 * @param msg The message to decode
 * @param xsens_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_xsens_attitude_decode(const mavlink_message_t* msg, mavlink_xsens_attitude_t* xsens_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	xsens_attitude->timestamp = mavlink_msg_xsens_attitude_get_timestamp(msg);
	xsens_attitude->roll = mavlink_msg_xsens_attitude_get_roll(msg);
	xsens_attitude->pitch = mavlink_msg_xsens_attitude_get_pitch(msg);
	xsens_attitude->yaw = mavlink_msg_xsens_attitude_get_yaw(msg);
	xsens_attitude->rollspeed = mavlink_msg_xsens_attitude_get_rollspeed(msg);
	xsens_attitude->pitchspeed = mavlink_msg_xsens_attitude_get_pitchspeed(msg);
	xsens_attitude->yawspeed = mavlink_msg_xsens_attitude_get_yawspeed(msg);
	xsens_attitude->rollacc = mavlink_msg_xsens_attitude_get_rollacc(msg);
	xsens_attitude->pitchacc = mavlink_msg_xsens_attitude_get_pitchacc(msg);
	xsens_attitude->yawacc = mavlink_msg_xsens_attitude_get_yawacc(msg);
#else
	memcpy(xsens_attitude, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_XSENS_ATTITUDE_LEN);
#endif
}
