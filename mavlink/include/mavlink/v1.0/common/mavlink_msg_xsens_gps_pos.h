// MESSAGE XSENS_GPS_POS PACKING

#define MAVLINK_MSG_ID_XSENS_GPS_POS 204

typedef struct __mavlink_xsens_gps_pos_t
{
 uint64_t timestamp_pos; ///< Timestamp for position information
 uint64_t timestamp_var; ///<  Timestamp for variance information
 uint64_t timestamp_v; ///< Timestamp for velocity informations
 int32_t lat; ///< Latitude in 1E-7 degrees
 int32_t lon; ///< Longitude in 1E-7 degrees
 int32_t alt; ///< Altitude in 1E-3 meters (millimeters) above MSL
 float var_s; ///<  speed accuracy estimate m/s
 float var_p; ///<  position accuracy estimate m
 float var_c; ///<  course accuracy estimate rad
 float eph_m; ///<  GPS HDOP horizontal dilution of position in m
 float epv_m; ///<  GPS VDOP horizontal dilution of position in m
 float vel_m_s; ///<  GPS ground speed (m/s)
 float vel_n_m_s; ///<  GPS ground speed (m/s)
 float vel_e_m_s; ///<  GPS ground speed (m/s)
 float vel_d_m_s; ///<  GPS ground speed (m/s)
 float cog_rad; ///<  Course over ground (NOT heading, but direction of movement) in rad, -PI..PI
} mavlink_xsens_gps_pos_t;

#define MAVLINK_MSG_ID_XSENS_GPS_POS_LEN 76
#define MAVLINK_MSG_ID_204_LEN 76

#define MAVLINK_MSG_ID_XSENS_GPS_POS_CRC 110
#define MAVLINK_MSG_ID_204_CRC 110



#define MAVLINK_MESSAGE_INFO_XSENS_GPS_POS { \
	"XSENS_GPS_POS", \
	16, \
	{  { "timestamp_pos", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_xsens_gps_pos_t, timestamp_pos) }, \
         { "timestamp_var", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_xsens_gps_pos_t, timestamp_var) }, \
         { "timestamp_v", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_xsens_gps_pos_t, timestamp_v) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_xsens_gps_pos_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_xsens_gps_pos_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_xsens_gps_pos_t, alt) }, \
         { "var_s", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_xsens_gps_pos_t, var_s) }, \
         { "var_p", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_xsens_gps_pos_t, var_p) }, \
         { "var_c", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_xsens_gps_pos_t, var_c) }, \
         { "eph_m", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_xsens_gps_pos_t, eph_m) }, \
         { "epv_m", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_xsens_gps_pos_t, epv_m) }, \
         { "vel_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_xsens_gps_pos_t, vel_m_s) }, \
         { "vel_n_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_xsens_gps_pos_t, vel_n_m_s) }, \
         { "vel_e_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_xsens_gps_pos_t, vel_e_m_s) }, \
         { "vel_d_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_xsens_gps_pos_t, vel_d_m_s) }, \
         { "cog_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_xsens_gps_pos_t, cog_rad) }, \
         } \
}


/**
 * @brief Pack a xsens_gps_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp_pos Timestamp for position information
 * @param lat Latitude in 1E-7 degrees
 * @param lon Longitude in 1E-7 degrees
 * @param alt Altitude in 1E-3 meters (millimeters) above MSL
 * @param timestamp_var  Timestamp for variance information
 * @param var_s  speed accuracy estimate m/s
 * @param var_p  position accuracy estimate m
 * @param var_c  course accuracy estimate rad
 * @param eph_m  GPS HDOP horizontal dilution of position in m
 * @param epv_m  GPS VDOP horizontal dilution of position in m
 * @param timestamp_v Timestamp for velocity informations
 * @param vel_m_s  GPS ground speed (m/s)
 * @param vel_n_m_s  GPS ground speed (m/s)
 * @param vel_e_m_s  GPS ground speed (m/s)
 * @param vel_d_m_s  GPS ground speed (m/s)
 * @param cog_rad  Course over ground (NOT heading, but direction of movement) in rad, -PI..PI
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_gps_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp_pos, int32_t lat, int32_t lon, int32_t alt, uint64_t timestamp_var, float var_s, float var_p, float var_c, float eph_m, float epv_m, uint64_t timestamp_v, float vel_m_s, float vel_n_m_s, float vel_e_m_s, float vel_d_m_s, float cog_rad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_GPS_POS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_pos);
	_mav_put_uint64_t(buf, 8, timestamp_var);
	_mav_put_uint64_t(buf, 16, timestamp_v);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, lon);
	_mav_put_int32_t(buf, 32, alt);
	_mav_put_float(buf, 36, var_s);
	_mav_put_float(buf, 40, var_p);
	_mav_put_float(buf, 44, var_c);
	_mav_put_float(buf, 48, eph_m);
	_mav_put_float(buf, 52, epv_m);
	_mav_put_float(buf, 56, vel_m_s);
	_mav_put_float(buf, 60, vel_n_m_s);
	_mav_put_float(buf, 64, vel_e_m_s);
	_mav_put_float(buf, 68, vel_d_m_s);
	_mav_put_float(buf, 72, cog_rad);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#else
	mavlink_xsens_gps_pos_t packet;
	packet.timestamp_pos = timestamp_pos;
	packet.timestamp_var = timestamp_var;
	packet.timestamp_v = timestamp_v;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.var_s = var_s;
	packet.var_p = var_p;
	packet.var_c = var_c;
	packet.eph_m = eph_m;
	packet.epv_m = epv_m;
	packet.vel_m_s = vel_m_s;
	packet.vel_n_m_s = vel_n_m_s;
	packet.vel_e_m_s = vel_e_m_s;
	packet.vel_d_m_s = vel_d_m_s;
	packet.cog_rad = cog_rad;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_GPS_POS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN, MAVLINK_MSG_ID_XSENS_GPS_POS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#endif
}

/**
 * @brief Pack a xsens_gps_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp_pos Timestamp for position information
 * @param lat Latitude in 1E-7 degrees
 * @param lon Longitude in 1E-7 degrees
 * @param alt Altitude in 1E-3 meters (millimeters) above MSL
 * @param timestamp_var  Timestamp for variance information
 * @param var_s  speed accuracy estimate m/s
 * @param var_p  position accuracy estimate m
 * @param var_c  course accuracy estimate rad
 * @param eph_m  GPS HDOP horizontal dilution of position in m
 * @param epv_m  GPS VDOP horizontal dilution of position in m
 * @param timestamp_v Timestamp for velocity informations
 * @param vel_m_s  GPS ground speed (m/s)
 * @param vel_n_m_s  GPS ground speed (m/s)
 * @param vel_e_m_s  GPS ground speed (m/s)
 * @param vel_d_m_s  GPS ground speed (m/s)
 * @param cog_rad  Course over ground (NOT heading, but direction of movement) in rad, -PI..PI
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xsens_gps_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp_pos,int32_t lat,int32_t lon,int32_t alt,uint64_t timestamp_var,float var_s,float var_p,float var_c,float eph_m,float epv_m,uint64_t timestamp_v,float vel_m_s,float vel_n_m_s,float vel_e_m_s,float vel_d_m_s,float cog_rad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_GPS_POS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_pos);
	_mav_put_uint64_t(buf, 8, timestamp_var);
	_mav_put_uint64_t(buf, 16, timestamp_v);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, lon);
	_mav_put_int32_t(buf, 32, alt);
	_mav_put_float(buf, 36, var_s);
	_mav_put_float(buf, 40, var_p);
	_mav_put_float(buf, 44, var_c);
	_mav_put_float(buf, 48, eph_m);
	_mav_put_float(buf, 52, epv_m);
	_mav_put_float(buf, 56, vel_m_s);
	_mav_put_float(buf, 60, vel_n_m_s);
	_mav_put_float(buf, 64, vel_e_m_s);
	_mav_put_float(buf, 68, vel_d_m_s);
	_mav_put_float(buf, 72, cog_rad);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#else
	mavlink_xsens_gps_pos_t packet;
	packet.timestamp_pos = timestamp_pos;
	packet.timestamp_var = timestamp_var;
	packet.timestamp_v = timestamp_v;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.var_s = var_s;
	packet.var_p = var_p;
	packet.var_c = var_c;
	packet.eph_m = eph_m;
	packet.epv_m = epv_m;
	packet.vel_m_s = vel_m_s;
	packet.vel_n_m_s = vel_n_m_s;
	packet.vel_e_m_s = vel_e_m_s;
	packet.vel_d_m_s = vel_d_m_s;
	packet.cog_rad = cog_rad;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XSENS_GPS_POS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN, MAVLINK_MSG_ID_XSENS_GPS_POS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#endif
}

/**
 * @brief Encode a xsens_gps_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param xsens_gps_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_gps_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_xsens_gps_pos_t* xsens_gps_pos)
{
	return mavlink_msg_xsens_gps_pos_pack(system_id, component_id, msg, xsens_gps_pos->timestamp_pos, xsens_gps_pos->lat, xsens_gps_pos->lon, xsens_gps_pos->alt, xsens_gps_pos->timestamp_var, xsens_gps_pos->var_s, xsens_gps_pos->var_p, xsens_gps_pos->var_c, xsens_gps_pos->eph_m, xsens_gps_pos->epv_m, xsens_gps_pos->timestamp_v, xsens_gps_pos->vel_m_s, xsens_gps_pos->vel_n_m_s, xsens_gps_pos->vel_e_m_s, xsens_gps_pos->vel_d_m_s, xsens_gps_pos->cog_rad);
}

/**
 * @brief Encode a xsens_gps_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xsens_gps_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xsens_gps_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_xsens_gps_pos_t* xsens_gps_pos)
{
	return mavlink_msg_xsens_gps_pos_pack_chan(system_id, component_id, chan, msg, xsens_gps_pos->timestamp_pos, xsens_gps_pos->lat, xsens_gps_pos->lon, xsens_gps_pos->alt, xsens_gps_pos->timestamp_var, xsens_gps_pos->var_s, xsens_gps_pos->var_p, xsens_gps_pos->var_c, xsens_gps_pos->eph_m, xsens_gps_pos->epv_m, xsens_gps_pos->timestamp_v, xsens_gps_pos->vel_m_s, xsens_gps_pos->vel_n_m_s, xsens_gps_pos->vel_e_m_s, xsens_gps_pos->vel_d_m_s, xsens_gps_pos->cog_rad);
}

/**
 * @brief Send a xsens_gps_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp_pos Timestamp for position information
 * @param lat Latitude in 1E-7 degrees
 * @param lon Longitude in 1E-7 degrees
 * @param alt Altitude in 1E-3 meters (millimeters) above MSL
 * @param timestamp_var  Timestamp for variance information
 * @param var_s  speed accuracy estimate m/s
 * @param var_p  position accuracy estimate m
 * @param var_c  course accuracy estimate rad
 * @param eph_m  GPS HDOP horizontal dilution of position in m
 * @param epv_m  GPS VDOP horizontal dilution of position in m
 * @param timestamp_v Timestamp for velocity informations
 * @param vel_m_s  GPS ground speed (m/s)
 * @param vel_n_m_s  GPS ground speed (m/s)
 * @param vel_e_m_s  GPS ground speed (m/s)
 * @param vel_d_m_s  GPS ground speed (m/s)
 * @param cog_rad  Course over ground (NOT heading, but direction of movement) in rad, -PI..PI
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_xsens_gps_pos_send(mavlink_channel_t chan, uint64_t timestamp_pos, int32_t lat, int32_t lon, int32_t alt, uint64_t timestamp_var, float var_s, float var_p, float var_c, float eph_m, float epv_m, uint64_t timestamp_v, float vel_m_s, float vel_n_m_s, float vel_e_m_s, float vel_d_m_s, float cog_rad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XSENS_GPS_POS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp_pos);
	_mav_put_uint64_t(buf, 8, timestamp_var);
	_mav_put_uint64_t(buf, 16, timestamp_v);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, lon);
	_mav_put_int32_t(buf, 32, alt);
	_mav_put_float(buf, 36, var_s);
	_mav_put_float(buf, 40, var_p);
	_mav_put_float(buf, 44, var_c);
	_mav_put_float(buf, 48, eph_m);
	_mav_put_float(buf, 52, epv_m);
	_mav_put_float(buf, 56, vel_m_s);
	_mav_put_float(buf, 60, vel_n_m_s);
	_mav_put_float(buf, 64, vel_e_m_s);
	_mav_put_float(buf, 68, vel_d_m_s);
	_mav_put_float(buf, 72, cog_rad);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GPS_POS, buf, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN, MAVLINK_MSG_ID_XSENS_GPS_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GPS_POS, buf, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#endif
#else
	mavlink_xsens_gps_pos_t packet;
	packet.timestamp_pos = timestamp_pos;
	packet.timestamp_var = timestamp_var;
	packet.timestamp_v = timestamp_v;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.var_s = var_s;
	packet.var_p = var_p;
	packet.var_c = var_c;
	packet.eph_m = eph_m;
	packet.epv_m = epv_m;
	packet.vel_m_s = vel_m_s;
	packet.vel_n_m_s = vel_n_m_s;
	packet.vel_e_m_s = vel_e_m_s;
	packet.vel_d_m_s = vel_d_m_s;
	packet.cog_rad = cog_rad;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GPS_POS, (const char *)&packet, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN, MAVLINK_MSG_ID_XSENS_GPS_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XSENS_GPS_POS, (const char *)&packet, MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#endif
#endif
}

#endif

// MESSAGE XSENS_GPS_POS UNPACKING


/**
 * @brief Get field timestamp_pos from xsens_gps_pos message
 *
 * @return Timestamp for position information
 */
static inline uint64_t mavlink_msg_xsens_gps_pos_get_timestamp_pos(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from xsens_gps_pos message
 *
 * @return Latitude in 1E-7 degrees
 */
static inline int32_t mavlink_msg_xsens_gps_pos_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field lon from xsens_gps_pos message
 *
 * @return Longitude in 1E-7 degrees
 */
static inline int32_t mavlink_msg_xsens_gps_pos_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field alt from xsens_gps_pos message
 *
 * @return Altitude in 1E-3 meters (millimeters) above MSL
 */
static inline int32_t mavlink_msg_xsens_gps_pos_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field timestamp_var from xsens_gps_pos message
 *
 * @return  Timestamp for variance information
 */
static inline uint64_t mavlink_msg_xsens_gps_pos_get_timestamp_var(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field var_s from xsens_gps_pos message
 *
 * @return  speed accuracy estimate m/s
 */
static inline float mavlink_msg_xsens_gps_pos_get_var_s(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field var_p from xsens_gps_pos message
 *
 * @return  position accuracy estimate m
 */
static inline float mavlink_msg_xsens_gps_pos_get_var_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field var_c from xsens_gps_pos message
 *
 * @return  course accuracy estimate rad
 */
static inline float mavlink_msg_xsens_gps_pos_get_var_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field eph_m from xsens_gps_pos message
 *
 * @return  GPS HDOP horizontal dilution of position in m
 */
static inline float mavlink_msg_xsens_gps_pos_get_eph_m(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field epv_m from xsens_gps_pos message
 *
 * @return  GPS VDOP horizontal dilution of position in m
 */
static inline float mavlink_msg_xsens_gps_pos_get_epv_m(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field timestamp_v from xsens_gps_pos message
 *
 * @return Timestamp for velocity informations
 */
static inline uint64_t mavlink_msg_xsens_gps_pos_get_timestamp_v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field vel_m_s from xsens_gps_pos message
 *
 * @return  GPS ground speed (m/s)
 */
static inline float mavlink_msg_xsens_gps_pos_get_vel_m_s(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field vel_n_m_s from xsens_gps_pos message
 *
 * @return  GPS ground speed (m/s)
 */
static inline float mavlink_msg_xsens_gps_pos_get_vel_n_m_s(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field vel_e_m_s from xsens_gps_pos message
 *
 * @return  GPS ground speed (m/s)
 */
static inline float mavlink_msg_xsens_gps_pos_get_vel_e_m_s(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field vel_d_m_s from xsens_gps_pos message
 *
 * @return  GPS ground speed (m/s)
 */
static inline float mavlink_msg_xsens_gps_pos_get_vel_d_m_s(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field cog_rad from xsens_gps_pos message
 *
 * @return  Course over ground (NOT heading, but direction of movement) in rad, -PI..PI
 */
static inline float mavlink_msg_xsens_gps_pos_get_cog_rad(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Decode a xsens_gps_pos message into a struct
 *
 * @param msg The message to decode
 * @param xsens_gps_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_xsens_gps_pos_decode(const mavlink_message_t* msg, mavlink_xsens_gps_pos_t* xsens_gps_pos)
{
#if MAVLINK_NEED_BYTE_SWAP
	xsens_gps_pos->timestamp_pos = mavlink_msg_xsens_gps_pos_get_timestamp_pos(msg);
	xsens_gps_pos->timestamp_var = mavlink_msg_xsens_gps_pos_get_timestamp_var(msg);
	xsens_gps_pos->timestamp_v = mavlink_msg_xsens_gps_pos_get_timestamp_v(msg);
	xsens_gps_pos->lat = mavlink_msg_xsens_gps_pos_get_lat(msg);
	xsens_gps_pos->lon = mavlink_msg_xsens_gps_pos_get_lon(msg);
	xsens_gps_pos->alt = mavlink_msg_xsens_gps_pos_get_alt(msg);
	xsens_gps_pos->var_s = mavlink_msg_xsens_gps_pos_get_var_s(msg);
	xsens_gps_pos->var_p = mavlink_msg_xsens_gps_pos_get_var_p(msg);
	xsens_gps_pos->var_c = mavlink_msg_xsens_gps_pos_get_var_c(msg);
	xsens_gps_pos->eph_m = mavlink_msg_xsens_gps_pos_get_eph_m(msg);
	xsens_gps_pos->epv_m = mavlink_msg_xsens_gps_pos_get_epv_m(msg);
	xsens_gps_pos->vel_m_s = mavlink_msg_xsens_gps_pos_get_vel_m_s(msg);
	xsens_gps_pos->vel_n_m_s = mavlink_msg_xsens_gps_pos_get_vel_n_m_s(msg);
	xsens_gps_pos->vel_e_m_s = mavlink_msg_xsens_gps_pos_get_vel_e_m_s(msg);
	xsens_gps_pos->vel_d_m_s = mavlink_msg_xsens_gps_pos_get_vel_d_m_s(msg);
	xsens_gps_pos->cog_rad = mavlink_msg_xsens_gps_pos_get_cog_rad(msg);
#else
	memcpy(xsens_gps_pos, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_XSENS_GPS_POS_LEN);
#endif
}
