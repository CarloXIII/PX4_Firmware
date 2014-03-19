// MESSAGE TWIST_ANGLE PACKING

#define MAVLINK_MSG_ID_TWIST_ANGLE 201

typedef struct __mavlink_twist_angle_t
{
 float Poti_Left; ///< si_value of the left poti 
 float Poti_Right; ///< si_value of the right poti 
 float Rel_Angle; ///< relative Angle between the potis 
} mavlink_twist_angle_t;

#define MAVLINK_MSG_ID_TWIST_ANGLE_LEN 12
#define MAVLINK_MSG_ID_201_LEN 12

#define MAVLINK_MSG_ID_TWIST_ANGLE_CRC 118
#define MAVLINK_MSG_ID_201_CRC 118



#define MAVLINK_MESSAGE_INFO_TWIST_ANGLE { \
	"TWIST_ANGLE", \
	3, \
	{  { "Poti_Left", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_twist_angle_t, Poti_Left) }, \
         { "Poti_Right", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_twist_angle_t, Poti_Right) }, \
         { "Rel_Angle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_twist_angle_t, Rel_Angle) }, \
         } \
}


/**
 * @brief Pack a twist_angle message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Poti_Left si_value of the left poti 
 * @param Poti_Right si_value of the right poti 
 * @param Rel_Angle relative Angle between the potis 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_twist_angle_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float Poti_Left, float Poti_Right, float Rel_Angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TWIST_ANGLE_LEN];
	_mav_put_float(buf, 0, Poti_Left);
	_mav_put_float(buf, 4, Poti_Right);
	_mav_put_float(buf, 8, Rel_Angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#else
	mavlink_twist_angle_t packet;
	packet.Poti_Left = Poti_Left;
	packet.Poti_Right = Poti_Right;
	packet.Rel_Angle = Rel_Angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TWIST_ANGLE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TWIST_ANGLE_LEN, MAVLINK_MSG_ID_TWIST_ANGLE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#endif
}

/**
 * @brief Pack a twist_angle message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Poti_Left si_value of the left poti 
 * @param Poti_Right si_value of the right poti 
 * @param Rel_Angle relative Angle between the potis 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_twist_angle_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float Poti_Left,float Poti_Right,float Rel_Angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TWIST_ANGLE_LEN];
	_mav_put_float(buf, 0, Poti_Left);
	_mav_put_float(buf, 4, Poti_Right);
	_mav_put_float(buf, 8, Rel_Angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#else
	mavlink_twist_angle_t packet;
	packet.Poti_Left = Poti_Left;
	packet.Poti_Right = Poti_Right;
	packet.Rel_Angle = Rel_Angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TWIST_ANGLE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TWIST_ANGLE_LEN, MAVLINK_MSG_ID_TWIST_ANGLE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#endif
}

/**
 * @brief Encode a twist_angle struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param twist_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_twist_angle_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_twist_angle_t* twist_angle)
{
	return mavlink_msg_twist_angle_pack(system_id, component_id, msg, twist_angle->Poti_Left, twist_angle->Poti_Right, twist_angle->Rel_Angle);
}

/**
 * @brief Encode a twist_angle struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param twist_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_twist_angle_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_twist_angle_t* twist_angle)
{
	return mavlink_msg_twist_angle_pack_chan(system_id, component_id, chan, msg, twist_angle->Poti_Left, twist_angle->Poti_Right, twist_angle->Rel_Angle);
}

/**
 * @brief Send a twist_angle message
 * @param chan MAVLink channel to send the message
 *
 * @param Poti_Left si_value of the left poti 
 * @param Poti_Right si_value of the right poti 
 * @param Rel_Angle relative Angle between the potis 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_twist_angle_send(mavlink_channel_t chan, float Poti_Left, float Poti_Right, float Rel_Angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TWIST_ANGLE_LEN];
	_mav_put_float(buf, 0, Poti_Left);
	_mav_put_float(buf, 4, Poti_Right);
	_mav_put_float(buf, 8, Rel_Angle);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TWIST_ANGLE, buf, MAVLINK_MSG_ID_TWIST_ANGLE_LEN, MAVLINK_MSG_ID_TWIST_ANGLE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TWIST_ANGLE, buf, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#endif
#else
	mavlink_twist_angle_t packet;
	packet.Poti_Left = Poti_Left;
	packet.Poti_Right = Poti_Right;
	packet.Rel_Angle = Rel_Angle;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TWIST_ANGLE, (const char *)&packet, MAVLINK_MSG_ID_TWIST_ANGLE_LEN, MAVLINK_MSG_ID_TWIST_ANGLE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TWIST_ANGLE, (const char *)&packet, MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#endif
#endif
}

#endif

// MESSAGE TWIST_ANGLE UNPACKING


/**
 * @brief Get field Poti_Left from twist_angle message
 *
 * @return si_value of the left poti 
 */
static inline float mavlink_msg_twist_angle_get_Poti_Left(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Poti_Right from twist_angle message
 *
 * @return si_value of the right poti 
 */
static inline float mavlink_msg_twist_angle_get_Poti_Right(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Rel_Angle from twist_angle message
 *
 * @return relative Angle between the potis 
 */
static inline float mavlink_msg_twist_angle_get_Rel_Angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a twist_angle message into a struct
 *
 * @param msg The message to decode
 * @param twist_angle C-struct to decode the message contents into
 */
static inline void mavlink_msg_twist_angle_decode(const mavlink_message_t* msg, mavlink_twist_angle_t* twist_angle)
{
#if MAVLINK_NEED_BYTE_SWAP
	twist_angle->Poti_Left = mavlink_msg_twist_angle_get_Poti_Left(msg);
	twist_angle->Poti_Right = mavlink_msg_twist_angle_get_Poti_Right(msg);
	twist_angle->Rel_Angle = mavlink_msg_twist_angle_get_Rel_Angle(msg);
#else
	memcpy(twist_angle, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TWIST_ANGLE_LEN);
#endif
}
