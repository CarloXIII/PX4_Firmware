// MESSAGE XSENS_SENS_RAW PACKING

#define MAVLINK_MSG_ID_XSENS_SENS_RAW 203

typedef struct __mavlink_xsens_sens_raw_t
{
 uint64_t timestamp; ///< Timestamp in microsecons since boot
 int16_t gyro_x; ///< X Raw sensor values of angular velocity
 int16_t gyro_y; ///< Y Raw sensor values of angular velocity
 int16_t gyro_z; ///< Z Raw sensor values of angular velocity
 int16_t acc_x; ///<  X Raw acceleration in NED body frame
 int16_t acc_y; ///<  Y Raw acceleration in NED body frame
 int16_t acc_z; ///<  Z Raw acceleration in NED body frame
 int16_t mag_x; ///<  X Raw magnetic field in NED body frame
 int16_t mag_y; ///<  Y Raw magnetic field in NED body frame
 int16_t mag_z; ///<  Z Raw magnetic field in NED body frame
} mavlink_xsens_sens_raw_t;

#define MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN 26
#define MAVLINK_MSG_ID_203_LEN 26

#define MAVLINK_MSG_ID_XSENS_SENS_RAW_CRC 46
#define MAVLINK_MSG_ID_203_CRC 46



#define MAVLINK_MESSAGE_INFO_XSENS_SENS_RAW { \
	"XSENS_SENS_RAW", \
	10, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_xsens_sens_raw_t, timestamp) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_xsens_sens_raw_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_xsens_sens_raw_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_xsens_sens_raw_t, gyro_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_xsens_sens_raw_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_xsens_sens_raw_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_xsens_sens_raw_t, acc_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_xsens_sens_raw_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_xsens_sens_raw_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_xsens_sens_raw_t, mag_z) }, \
         } \
}


/**
 * @brief Pack a xsens_sens_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp in microsecons since boot
 * @param gyro_x X Raw sensor values of angular velocity
 * @param gyro_y Y Raw sensor values of angular velocity
 * @param gyro_z Z Raw sensor values of angular velocity
 * @param acc_x  X Raw acceleration in NED body frame
 * @param acc_y  Y Raw acceleration in NED body frame
 * @param acc_z  Z Raw acceleration in NED body frame
 * @param mag_x  X Raw magnetic field in NED body frame
 * @param mag_y  Y Raw magnetic field in NED body frame
 * @param mag_z  Z Raw magnetic field in NED body frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_sens_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int16_t(buf, 8, gyro_x);
	_mav_put_int16_t(buf, 10, gyro_y);
	_mav_put_int16_t(buf, 12, gyro_z);
	_mav_put_int16_t(buf, 14, acc_x);
	_mav_put_int16_t(buf, 16, acc_y);
	_mav_put_int16_t(buf, 18, acc_z);
	_mav_put_int16_t(buf, 20, mag_x);
	_mav_put_int16_t(buf, 22, mag_y);
	_mav_put_int16_t(buf, 24, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#else
	mavlink_xsens_sens_raw_t packet;
	packet.timestamp = timestamp;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;
	packet.acc_x = acc_x;
	packet.acc_y = acc_y;
	packet.acc_z = acc_z;
	packet.mag_x = mag_x;
	packet.mag_y = mag_y;
	packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_SENS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN, MAVLINK_MSG_ID_XSENS_SENS_RAW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#endif
}

/**
 * @brief Pack a xsens_sens_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp in microsecons since boot
 * @param gyro_x X Raw sensor values of angular velocity
 * @param gyro_y Y Raw sensor values of angular velocity
 * @param gyro_z Z Raw sensor values of angular velocity
 * @param acc_x  X Raw acceleration in NED body frame
 * @param acc_y  Y Raw acceleration in NED body frame
 * @param acc_z  Z Raw acceleration in NED body frame
 * @param mag_x  X Raw magnetic field in NED body frame
 * @param mag_y  Y Raw magnetic field in NED body frame
 * @param mag_z  Z Raw magnetic field in NED body frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_sens_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,int16_t gyro_x,int16_t gyro_y,int16_t gyro_z,int16_t acc_x,int16_t acc_y,int16_t acc_z,int16_t mag_x,int16_t mag_y,int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int16_t(buf, 8, gyro_x);
	_mav_put_int16_t(buf, 10, gyro_y);
	_mav_put_int16_t(buf, 12, gyro_z);
	_mav_put_int16_t(buf, 14, acc_x);
	_mav_put_int16_t(buf, 16, acc_y);
	_mav_put_int16_t(buf, 18, acc_z);
	_mav_put_int16_t(buf, 20, mag_x);
	_mav_put_int16_t(buf, 22, mag_y);
	_mav_put_int16_t(buf, 24, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#else
	mavlink_xsens_sens_raw_t packet;
	packet.timestamp = timestamp;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;
	packet.acc_x = acc_x;
	packet.acc_y = acc_y;
	packet.acc_z = acc_z;
	packet.mag_x = mag_x;
	packet.mag_y = mag_y;
	packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_SENS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN, MAVLINK_MSG_ID_XSENS_SENS_RAW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#endif
}

/**
 * @brief Encode a xsens_sens_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param xsens_sens_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_sens_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_xsens_sens_raw_t* xsens_sens_raw)
{
	return mavlink_msg_xsens_sens_raw_pack(system_id, component_id, msg, xsens_sens_raw->timestamp, xsens_sens_raw->gyro_x, xsens_sens_raw->gyro_y, xsens_sens_raw->gyro_z, xsens_sens_raw->acc_x, xsens_sens_raw->acc_y, xsens_sens_raw->acc_z, xsens_sens_raw->mag_x, xsens_sens_raw->mag_y, xsens_sens_raw->mag_z);
}

/**
 * @brief Encode a xsens_sens_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xsens_sens_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_sens_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_xsens_sens_raw_t* xsens_sens_raw)
{
	return mavlink_msg_xsens_sens_raw_pack_chan(system_id, component_id, chan, msg, xsens_sens_raw->timestamp, xsens_sens_raw->gyro_x, xsens_sens_raw->gyro_y, xsens_sens_raw->gyro_z, xsens_sens_raw->acc_x, xsens_sens_raw->acc_y, xsens_sens_raw->acc_z, xsens_sens_raw->mag_x, xsens_sens_raw->mag_y, xsens_sens_raw->mag_z);
}

/**
 * @brief Send a xsens_sens_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp in microsecons since boot
 * @param gyro_x X Raw sensor values of angular velocity
 * @param gyro_y Y Raw sensor values of angular velocity
 * @param gyro_z Z Raw sensor values of angular velocity
 * @param acc_x  X Raw acceleration in NED body frame
 * @param acc_y  Y Raw acceleration in NED body frame
 * @param acc_z  Z Raw acceleration in NED body frame
 * @param mag_x  X Raw magnetic field in NED body frame
 * @param mag_y  Y Raw magnetic field in NED body frame
 * @param mag_z  Z Raw magnetic field in NED body frame
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_xsens_sens_raw_send(mavlink_channel_t chan, uint64_t timestamp, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int16_t(buf, 8, gyro_x);
	_mav_put_int16_t(buf, 10, gyro_y);
	_mav_put_int16_t(buf, 12, gyro_z);
	_mav_put_int16_t(buf, 14, acc_x);
	_mav_put_int16_t(buf, 16, acc_y);
	_mav_put_int16_t(buf, 18, acc_z);
	_mav_put_int16_t(buf, 20, mag_x);
	_mav_put_int16_t(buf, 22, mag_y);
	_mav_put_int16_t(buf, 24, mag_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_RAW, buf, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN, MAVLINK_MSG_ID_XSENS_SENS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_RAW, buf, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#endif
#else
	mavlink_xsens_sens_raw_t packet;
	packet.timestamp = timestamp;
	packet.gyro_x = gyro_x;
	packet.gyro_y = gyro_y;
	packet.gyro_z = gyro_z;
	packet.acc_x = acc_x;
	packet.acc_y = acc_y;
	packet.acc_z = acc_z;
	packet.mag_x = mag_x;
	packet.mag_y = mag_y;
	packet.mag_z = mag_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_RAW, (const char *)&packet, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN, MAVLINK_MSG_ID_XSENS_SENS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_RAW, (const char *)&packet, MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#endif
#endif
}

#endif

// MESSAGE XSENS_SENS_RAW UNPACKING


/**
 * @brief Get field timestamp from xsens_sens_raw message
 *
 * @return Timestamp in microsecons since boot
 */
static inline uint64_t mavlink_msg_xsens_sens_raw_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gyro_x from xsens_sens_raw message
 *
 * @return X Raw sensor values of angular velocity
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_gyro_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field gyro_y from xsens_sens_raw message
 *
 * @return Y Raw sensor values of angular velocity
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_gyro_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field gyro_z from xsens_sens_raw message
 *
 * @return Z Raw sensor values of angular velocity
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_gyro_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field acc_x from xsens_sens_raw message
 *
 * @return  X Raw acceleration in NED body frame
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_acc_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field acc_y from xsens_sens_raw message
 *
 * @return  Y Raw acceleration in NED body frame
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_acc_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field acc_z from xsens_sens_raw message
 *
 * @return  Z Raw acceleration in NED body frame
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_acc_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field mag_x from xsens_sens_raw message
 *
 * @return  X Raw magnetic field in NED body frame
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_mag_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field mag_y from xsens_sens_raw message
 *
 * @return  Y Raw magnetic field in NED body frame
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_mag_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field mag_z from xsens_sens_raw message
 *
 * @return  Z Raw magnetic field in NED body frame
 */
static inline int16_t mavlink_msg_xsens_sens_raw_get_mag_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Decode a xsens_sens_raw message into a struct
 *
 * @param msg The message to decode
 * @param xsens_sens_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_xsens_sens_raw_decode(const mavlink_message_t* msg, mavlink_xsens_sens_raw_t* xsens_sens_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	xsens_sens_raw->timestamp = mavlink_msg_xsens_sens_raw_get_timestamp(msg);
	xsens_sens_raw->gyro_x = mavlink_msg_xsens_sens_raw_get_gyro_x(msg);
	xsens_sens_raw->gyro_y = mavlink_msg_xsens_sens_raw_get_gyro_y(msg);
	xsens_sens_raw->gyro_z = mavlink_msg_xsens_sens_raw_get_gyro_z(msg);
	xsens_sens_raw->acc_x = mavlink_msg_xsens_sens_raw_get_acc_x(msg);
	xsens_sens_raw->acc_y = mavlink_msg_xsens_sens_raw_get_acc_y(msg);
	xsens_sens_raw->acc_z = mavlink_msg_xsens_sens_raw_get_acc_z(msg);
	xsens_sens_raw->mag_x = mavlink_msg_xsens_sens_raw_get_mag_x(msg);
	xsens_sens_raw->mag_y = mavlink_msg_xsens_sens_raw_get_mag_y(msg);
	xsens_sens_raw->mag_z = mavlink_msg_xsens_sens_raw_get_mag_z(msg);
#else
	memcpy(xsens_sens_raw, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_XSENS_SENS_RAW_LEN);
#endif
}
