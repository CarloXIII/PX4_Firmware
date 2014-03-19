// MESSAGE XSENS_SENS_COMB PACKING

#define MAVLINK_MSG_ID_XSENS_SENS_COMB 202

typedef struct __mavlink_xsens_sens_comb_t
{
 uint64_t timestamp; ///< Timestamp in microsecons since boot
 float gyro_x; ///< X Angular velocity (rad/s)
 float gyro_y; ///< Y Angular velocity (rad/s)
 float gyro_z; ///< Z Angular velocity (rad/s)
 float acc_x; ///<  X Acceleration in NED body frame, in m/s^2
 float acc_y; ///<  Y Acceleration in NED body frame, in m/s^2
 float acc_z; ///<  Z Acceleration in NED body frame, in m/s^2
 float mag_x; ///<  X Magnetic field in NED body frame, in Gauss
 float mag_y; ///<  Y Magnetic field in NED body frame, in Gauss
 float mag_z; ///<  Z Magnetic field in NED body frame, in Gauss
 float baro_pres; ///< Barometric pressure, already temp. comp.
 float baro_alt; ///< Altitude, already temp. comp.
 float baro_temp; ///< Temperature in degrees celsius
 float diff_pres; ///< Airspeed sensor differential pressure
} mavlink_xsens_sens_comb_t;

#define MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN 60
#define MAVLINK_MSG_ID_202_LEN 60

#define MAVLINK_MSG_ID_XSENS_SENS_COMB_CRC 145
#define MAVLINK_MSG_ID_202_CRC 145



#define MAVLINK_MESSAGE_INFO_XSENS_SENS_COMB { \
	"XSENS_SENS_COMB", \
	14, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_xsens_sens_comb_t, timestamp) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_xsens_sens_comb_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_xsens_sens_comb_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_xsens_sens_comb_t, gyro_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_xsens_sens_comb_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_xsens_sens_comb_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_xsens_sens_comb_t, acc_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_xsens_sens_comb_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_xsens_sens_comb_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_xsens_sens_comb_t, mag_z) }, \
         { "baro_pres", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_xsens_sens_comb_t, baro_pres) }, \
         { "baro_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_xsens_sens_comb_t, baro_alt) }, \
         { "baro_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_xsens_sens_comb_t, baro_temp) }, \
         { "diff_pres", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_xsens_sens_comb_t, diff_pres) }, \
         } \
}


/**
 * @brief Pack a xsens_sens_comb message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp in microsecons since boot
 * @param gyro_x X Angular velocity (rad/s)
 * @param gyro_y Y Angular velocity (rad/s)
 * @param gyro_z Z Angular velocity (rad/s)
 * @param acc_x  X Acceleration in NED body frame, in m/s^2
 * @param acc_y  Y Acceleration in NED body frame, in m/s^2
 * @param acc_z  Z Acceleration in NED body frame, in m/s^2
 * @param mag_x  X Magnetic field in NED body frame, in Gauss
 * @param mag_y  Y Magnetic field in NED body frame, in Gauss
 * @param mag_z  Z Magnetic field in NED body frame, in Gauss
 * @param baro_pres Barometric pressure, already temp. comp.
 * @param baro_alt Altitude, already temp. comp.
 * @param baro_temp Temperature in degrees celsius
 * @param diff_pres Airspeed sensor differential pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_sens_comb_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z, float mag_x, float mag_y, float mag_z, float baro_pres, float baro_alt, float baro_temp, float diff_pres)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, gyro_x);
	_mav_put_float(buf, 12, gyro_y);
	_mav_put_float(buf, 16, gyro_z);
	_mav_put_float(buf, 20, acc_x);
	_mav_put_float(buf, 24, acc_y);
	_mav_put_float(buf, 28, acc_z);
	_mav_put_float(buf, 32, mag_x);
	_mav_put_float(buf, 36, mag_y);
	_mav_put_float(buf, 40, mag_z);
	_mav_put_float(buf, 44, baro_pres);
	_mav_put_float(buf, 48, baro_alt);
	_mav_put_float(buf, 52, baro_temp);
	_mav_put_float(buf, 56, diff_pres);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#else
	mavlink_xsens_sens_comb_t packet;
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
	packet.baro_pres = baro_pres;
	packet.baro_alt = baro_alt;
	packet.baro_temp = baro_temp;
	packet.diff_pres = diff_pres;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_SENS_COMB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN, MAVLINK_MSG_ID_XSENS_SENS_COMB_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#endif
}

/**
 * @brief Pack a xsens_sens_comb message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp in microsecons since boot
 * @param gyro_x X Angular velocity (rad/s)
 * @param gyro_y Y Angular velocity (rad/s)
 * @param gyro_z Z Angular velocity (rad/s)
 * @param acc_x  X Acceleration in NED body frame, in m/s^2
 * @param acc_y  Y Acceleration in NED body frame, in m/s^2
 * @param acc_z  Z Acceleration in NED body frame, in m/s^2
 * @param mag_x  X Magnetic field in NED body frame, in Gauss
 * @param mag_y  Y Magnetic field in NED body frame, in Gauss
 * @param mag_z  Z Magnetic field in NED body frame, in Gauss
 * @param baro_pres Barometric pressure, already temp. comp.
 * @param baro_alt Altitude, already temp. comp.
 * @param baro_temp Temperature in degrees celsius
 * @param diff_pres Airspeed sensor differential pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_sens_comb_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float gyro_x,float gyro_y,float gyro_z,float acc_x,float acc_y,float acc_z,float mag_x,float mag_y,float mag_z,float baro_pres,float baro_alt,float baro_temp,float diff_pres)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, gyro_x);
	_mav_put_float(buf, 12, gyro_y);
	_mav_put_float(buf, 16, gyro_z);
	_mav_put_float(buf, 20, acc_x);
	_mav_put_float(buf, 24, acc_y);
	_mav_put_float(buf, 28, acc_z);
	_mav_put_float(buf, 32, mag_x);
	_mav_put_float(buf, 36, mag_y);
	_mav_put_float(buf, 40, mag_z);
	_mav_put_float(buf, 44, baro_pres);
	_mav_put_float(buf, 48, baro_alt);
	_mav_put_float(buf, 52, baro_temp);
	_mav_put_float(buf, 56, diff_pres);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#else
	mavlink_xsens_sens_comb_t packet;
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
	packet.baro_pres = baro_pres;
	packet.baro_alt = baro_alt;
	packet.baro_temp = baro_temp;
	packet.diff_pres = diff_pres;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_SENS_COMB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN, MAVLINK_MSG_ID_XSENS_SENS_COMB_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#endif
}

/**
 * @brief Encode a xsens_sens_comb struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param xsens_sens_comb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_sens_comb_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_xsens_sens_comb_t* xsens_sens_comb)
{
	return mavlink_msg_xsens_sens_comb_pack(system_id, component_id, msg, xsens_sens_comb->timestamp, xsens_sens_comb->gyro_x, xsens_sens_comb->gyro_y, xsens_sens_comb->gyro_z, xsens_sens_comb->acc_x, xsens_sens_comb->acc_y, xsens_sens_comb->acc_z, xsens_sens_comb->mag_x, xsens_sens_comb->mag_y, xsens_sens_comb->mag_z, xsens_sens_comb->baro_pres, xsens_sens_comb->baro_alt, xsens_sens_comb->baro_temp, xsens_sens_comb->diff_pres);
}

/**
 * @brief Encode a xsens_sens_comb struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xsens_sens_comb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_sens_comb_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_xsens_sens_comb_t* xsens_sens_comb)
{
	return mavlink_msg_xsens_sens_comb_pack_chan(system_id, component_id, chan, msg, xsens_sens_comb->timestamp, xsens_sens_comb->gyro_x, xsens_sens_comb->gyro_y, xsens_sens_comb->gyro_z, xsens_sens_comb->acc_x, xsens_sens_comb->acc_y, xsens_sens_comb->acc_z, xsens_sens_comb->mag_x, xsens_sens_comb->mag_y, xsens_sens_comb->mag_z, xsens_sens_comb->baro_pres, xsens_sens_comb->baro_alt, xsens_sens_comb->baro_temp, xsens_sens_comb->diff_pres);
}

/**
 * @brief Send a xsens_sens_comb message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp in microsecons since boot
 * @param gyro_x X Angular velocity (rad/s)
 * @param gyro_y Y Angular velocity (rad/s)
 * @param gyro_z Z Angular velocity (rad/s)
 * @param acc_x  X Acceleration in NED body frame, in m/s^2
 * @param acc_y  Y Acceleration in NED body frame, in m/s^2
 * @param acc_z  Z Acceleration in NED body frame, in m/s^2
 * @param mag_x  X Magnetic field in NED body frame, in Gauss
 * @param mag_y  Y Magnetic field in NED body frame, in Gauss
 * @param mag_z  Z Magnetic field in NED body frame, in Gauss
 * @param baro_pres Barometric pressure, already temp. comp.
 * @param baro_alt Altitude, already temp. comp.
 * @param baro_temp Temperature in degrees celsius
 * @param diff_pres Airspeed sensor differential pressure
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_xsens_sens_comb_send(mavlink_channel_t chan, uint64_t timestamp, float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z, float mag_x, float mag_y, float mag_z, float baro_pres, float baro_alt, float baro_temp, float diff_pres)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, gyro_x);
	_mav_put_float(buf, 12, gyro_y);
	_mav_put_float(buf, 16, gyro_z);
	_mav_put_float(buf, 20, acc_x);
	_mav_put_float(buf, 24, acc_y);
	_mav_put_float(buf, 28, acc_z);
	_mav_put_float(buf, 32, mag_x);
	_mav_put_float(buf, 36, mag_y);
	_mav_put_float(buf, 40, mag_z);
	_mav_put_float(buf, 44, baro_pres);
	_mav_put_float(buf, 48, baro_alt);
	_mav_put_float(buf, 52, baro_temp);
	_mav_put_float(buf, 56, diff_pres);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_COMB, buf, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN, MAVLINK_MSG_ID_XSENS_SENS_COMB_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_COMB, buf, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#endif
#else
	mavlink_xsens_sens_comb_t packet;
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
	packet.baro_pres = baro_pres;
	packet.baro_alt = baro_alt;
	packet.baro_temp = baro_temp;
	packet.diff_pres = diff_pres;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_COMB, (const char *)&packet, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN, MAVLINK_MSG_ID_XSENS_SENS_COMB_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_SENS_COMB, (const char *)&packet, MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#endif
#endif
}

#endif

// MESSAGE XSENS_SENS_COMB UNPACKING


/**
 * @brief Get field timestamp from xsens_sens_comb message
 *
 * @return Timestamp in microsecons since boot
 */
static inline uint64_t mavlink_msg_xsens_sens_comb_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gyro_x from xsens_sens_comb message
 *
 * @return X Angular velocity (rad/s)
 */
static inline float mavlink_msg_xsens_sens_comb_get_gyro_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gyro_y from xsens_sens_comb message
 *
 * @return Y Angular velocity (rad/s)
 */
static inline float mavlink_msg_xsens_sens_comb_get_gyro_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_z from xsens_sens_comb message
 *
 * @return Z Angular velocity (rad/s)
 */
static inline float mavlink_msg_xsens_sens_comb_get_gyro_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field acc_x from xsens_sens_comb message
 *
 * @return  X Acceleration in NED body frame, in m/s^2
 */
static inline float mavlink_msg_xsens_sens_comb_get_acc_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field acc_y from xsens_sens_comb message
 *
 * @return  Y Acceleration in NED body frame, in m/s^2
 */
static inline float mavlink_msg_xsens_sens_comb_get_acc_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field acc_z from xsens_sens_comb message
 *
 * @return  Z Acceleration in NED body frame, in m/s^2
 */
static inline float mavlink_msg_xsens_sens_comb_get_acc_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field mag_x from xsens_sens_comb message
 *
 * @return  X Magnetic field in NED body frame, in Gauss
 */
static inline float mavlink_msg_xsens_sens_comb_get_mag_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field mag_y from xsens_sens_comb message
 *
 * @return  Y Magnetic field in NED body frame, in Gauss
 */
static inline float mavlink_msg_xsens_sens_comb_get_mag_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field mag_z from xsens_sens_comb message
 *
 * @return  Z Magnetic field in NED body frame, in Gauss
 */
static inline float mavlink_msg_xsens_sens_comb_get_mag_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field baro_pres from xsens_sens_comb message
 *
 * @return Barometric pressure, already temp. comp.
 */
static inline float mavlink_msg_xsens_sens_comb_get_baro_pres(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field baro_alt from xsens_sens_comb message
 *
 * @return Altitude, already temp. comp.
 */
static inline float mavlink_msg_xsens_sens_comb_get_baro_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field baro_temp from xsens_sens_comb message
 *
 * @return Temperature in degrees celsius
 */
static inline float mavlink_msg_xsens_sens_comb_get_baro_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field diff_pres from xsens_sens_comb message
 *
 * @return Airspeed sensor differential pressure
 */
static inline float mavlink_msg_xsens_sens_comb_get_diff_pres(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Decode a xsens_sens_comb message into a struct
 *
 * @param msg The message to decode
 * @param xsens_sens_comb C-struct to decode the message contents into
 */
static inline void mavlink_msg_xsens_sens_comb_decode(const mavlink_message_t* msg, mavlink_xsens_sens_comb_t* xsens_sens_comb)
{
#if MAVLINK_NEED_BYTE_SWAP
	xsens_sens_comb->timestamp = mavlink_msg_xsens_sens_comb_get_timestamp(msg);
	xsens_sens_comb->gyro_x = mavlink_msg_xsens_sens_comb_get_gyro_x(msg);
	xsens_sens_comb->gyro_y = mavlink_msg_xsens_sens_comb_get_gyro_y(msg);
	xsens_sens_comb->gyro_z = mavlink_msg_xsens_sens_comb_get_gyro_z(msg);
	xsens_sens_comb->acc_x = mavlink_msg_xsens_sens_comb_get_acc_x(msg);
	xsens_sens_comb->acc_y = mavlink_msg_xsens_sens_comb_get_acc_y(msg);
	xsens_sens_comb->acc_z = mavlink_msg_xsens_sens_comb_get_acc_z(msg);
	xsens_sens_comb->mag_x = mavlink_msg_xsens_sens_comb_get_mag_x(msg);
	xsens_sens_comb->mag_y = mavlink_msg_xsens_sens_comb_get_mag_y(msg);
	xsens_sens_comb->mag_z = mavlink_msg_xsens_sens_comb_get_mag_z(msg);
	xsens_sens_comb->baro_pres = mavlink_msg_xsens_sens_comb_get_baro_pres(msg);
	xsens_sens_comb->baro_alt = mavlink_msg_xsens_sens_comb_get_baro_alt(msg);
	xsens_sens_comb->baro_temp = mavlink_msg_xsens_sens_comb_get_baro_temp(msg);
	xsens_sens_comb->diff_pres = mavlink_msg_xsens_sens_comb_get_diff_pres(msg);
#else
	memcpy(xsens_sens_comb, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_XSENS_SENS_COMB_LEN);
#endif
}
