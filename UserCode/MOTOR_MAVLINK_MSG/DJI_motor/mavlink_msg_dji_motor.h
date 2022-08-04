#pragma once
// MESSAGE DJI_MOTOR PACKING

#define MAVLINK_MSG_ID_DJI_MOTOR 42


typedef struct __mavlink_dji_motor_t {
 float angle_fdb; /*<  angle_fdb*/
 float rpm_fdb; /*<  rpm_fdb*/
 float current_fdb; /*<  current_fdb*/
 float current_out; /*<  current_out*/
 int8_t id; /*<  motor id*/
} mavlink_dji_motor_t;

#define MAVLINK_MSG_ID_DJI_MOTOR_LEN 17
#define MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN 17
#define MAVLINK_MSG_ID_42_LEN 17
#define MAVLINK_MSG_ID_42_MIN_LEN 17

#define MAVLINK_MSG_ID_DJI_MOTOR_CRC 105
#define MAVLINK_MSG_ID_42_CRC 105



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DJI_MOTOR { \
    42, \
    "DJI_MOTOR", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_INT8_T, 0, 16, offsetof(mavlink_dji_motor_t, id) }, \
         { "angle_fdb", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dji_motor_t, angle_fdb) }, \
         { "rpm_fdb", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dji_motor_t, rpm_fdb) }, \
         { "current_fdb", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dji_motor_t, current_fdb) }, \
         { "current_out", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_dji_motor_t, current_out) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DJI_MOTOR { \
    "DJI_MOTOR", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_INT8_T, 0, 16, offsetof(mavlink_dji_motor_t, id) }, \
         { "angle_fdb", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dji_motor_t, angle_fdb) }, \
         { "rpm_fdb", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dji_motor_t, rpm_fdb) }, \
         { "current_fdb", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dji_motor_t, current_fdb) }, \
         { "current_out", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_dji_motor_t, current_out) }, \
         } \
}
#endif

/**
 * @brief Pack a dji_motor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  motor id
 * @param angle_fdb  angle_fdb
 * @param rpm_fdb  rpm_fdb
 * @param current_fdb  current_fdb
 * @param current_out  current_out
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dji_motor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t id, float angle_fdb, float rpm_fdb, float current_fdb, float current_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DJI_MOTOR_LEN];
    _mav_put_float(buf, 0, angle_fdb);
    _mav_put_float(buf, 4, rpm_fdb);
    _mav_put_float(buf, 8, current_fdb);
    _mav_put_float(buf, 12, current_out);
    _mav_put_int8_t(buf, 16, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DJI_MOTOR_LEN);
#else
    mavlink_dji_motor_t packet;
    packet.angle_fdb = angle_fdb;
    packet.rpm_fdb = rpm_fdb;
    packet.current_fdb = current_fdb;
    packet.current_out = current_out;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DJI_MOTOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DJI_MOTOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN, MAVLINK_MSG_ID_DJI_MOTOR_LEN, MAVLINK_MSG_ID_DJI_MOTOR_CRC);
}

/**
 * @brief Pack a dji_motor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  motor id
 * @param angle_fdb  angle_fdb
 * @param rpm_fdb  rpm_fdb
 * @param current_fdb  current_fdb
 * @param current_out  current_out
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dji_motor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t id,float angle_fdb,float rpm_fdb,float current_fdb,float current_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DJI_MOTOR_LEN];
    _mav_put_float(buf, 0, angle_fdb);
    _mav_put_float(buf, 4, rpm_fdb);
    _mav_put_float(buf, 8, current_fdb);
    _mav_put_float(buf, 12, current_out);
    _mav_put_int8_t(buf, 16, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DJI_MOTOR_LEN);
#else
    mavlink_dji_motor_t packet;
    packet.angle_fdb = angle_fdb;
    packet.rpm_fdb = rpm_fdb;
    packet.current_fdb = current_fdb;
    packet.current_out = current_out;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DJI_MOTOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DJI_MOTOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN, MAVLINK_MSG_ID_DJI_MOTOR_LEN, MAVLINK_MSG_ID_DJI_MOTOR_CRC);
}

/**
 * @brief Encode a dji_motor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dji_motor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dji_motor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dji_motor_t* dji_motor)
{
    return mavlink_msg_dji_motor_pack(system_id, component_id, msg, dji_motor->id, dji_motor->angle_fdb, dji_motor->rpm_fdb, dji_motor->current_fdb, dji_motor->current_out);
}

/**
 * @brief Encode a dji_motor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dji_motor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dji_motor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dji_motor_t* dji_motor)
{
    return mavlink_msg_dji_motor_pack_chan(system_id, component_id, chan, msg, dji_motor->id, dji_motor->angle_fdb, dji_motor->rpm_fdb, dji_motor->current_fdb, dji_motor->current_out);
}

/**
 * @brief Send a dji_motor message
 * @param chan MAVLink channel to send the message
 *
 * @param id  motor id
 * @param angle_fdb  angle_fdb
 * @param rpm_fdb  rpm_fdb
 * @param current_fdb  current_fdb
 * @param current_out  current_out
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dji_motor_send(mavlink_channel_t chan, int8_t id, float angle_fdb, float rpm_fdb, float current_fdb, float current_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DJI_MOTOR_LEN];
    _mav_put_float(buf, 0, angle_fdb);
    _mav_put_float(buf, 4, rpm_fdb);
    _mav_put_float(buf, 8, current_fdb);
    _mav_put_float(buf, 12, current_out);
    _mav_put_int8_t(buf, 16, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DJI_MOTOR, buf, MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN, MAVLINK_MSG_ID_DJI_MOTOR_LEN, MAVLINK_MSG_ID_DJI_MOTOR_CRC);
#else
    mavlink_dji_motor_t packet;
    packet.angle_fdb = angle_fdb;
    packet.rpm_fdb = rpm_fdb;
    packet.current_fdb = current_fdb;
    packet.current_out = current_out;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DJI_MOTOR, (const char *)&packet, MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN, MAVLINK_MSG_ID_DJI_MOTOR_LEN, MAVLINK_MSG_ID_DJI_MOTOR_CRC);
#endif
}

/**
 * @brief Send a dji_motor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dji_motor_send_struct(mavlink_channel_t chan, const mavlink_dji_motor_t* dji_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dji_motor_send(chan, dji_motor->id, dji_motor->angle_fdb, dji_motor->rpm_fdb, dji_motor->current_fdb, dji_motor->current_out);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DJI_MOTOR, (const char *)dji_motor, MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN, MAVLINK_MSG_ID_DJI_MOTOR_LEN, MAVLINK_MSG_ID_DJI_MOTOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_DJI_MOTOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dji_motor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t id, float angle_fdb, float rpm_fdb, float current_fdb, float current_out)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, angle_fdb);
    _mav_put_float(buf, 4, rpm_fdb);
    _mav_put_float(buf, 8, current_fdb);
    _mav_put_float(buf, 12, current_out);
    _mav_put_int8_t(buf, 16, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DJI_MOTOR, buf, MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN, MAVLINK_MSG_ID_DJI_MOTOR_LEN, MAVLINK_MSG_ID_DJI_MOTOR_CRC);
#else
    mavlink_dji_motor_t *packet = (mavlink_dji_motor_t *)msgbuf;
    packet->angle_fdb = angle_fdb;
    packet->rpm_fdb = rpm_fdb;
    packet->current_fdb = current_fdb;
    packet->current_out = current_out;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DJI_MOTOR, (const char *)packet, MAVLINK_MSG_ID_DJI_MOTOR_MIN_LEN, MAVLINK_MSG_ID_DJI_MOTOR_LEN, MAVLINK_MSG_ID_DJI_MOTOR_CRC);
#endif
}
#endif

#endif

// MESSAGE DJI_MOTOR UNPACKING


/**
 * @brief Get field id from dji_motor message
 *
 * @return  motor id
 */
static inline int8_t mavlink_msg_dji_motor_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  16);
}

/**
 * @brief Get field angle_fdb from dji_motor message
 *
 * @return  angle_fdb
 */
static inline float mavlink_msg_dji_motor_get_angle_fdb(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field rpm_fdb from dji_motor message
 *
 * @return  rpm_fdb
 */
static inline float mavlink_msg_dji_motor_get_rpm_fdb(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field current_fdb from dji_motor message
 *
 * @return  current_fdb
 */
static inline float mavlink_msg_dji_motor_get_current_fdb(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field current_out from dji_motor message
 *
 * @return  current_out
 */
static inline float mavlink_msg_dji_motor_get_current_out(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a dji_motor message into a struct
 *
 * @param msg The message to decode
 * @param dji_motor C-struct to decode the message contents into
 */
static inline void mavlink_msg_dji_motor_decode(const mavlink_message_t* msg, mavlink_dji_motor_t* dji_motor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dji_motor->angle_fdb = mavlink_msg_dji_motor_get_angle_fdb(msg);
    dji_motor->rpm_fdb = mavlink_msg_dji_motor_get_rpm_fdb(msg);
    dji_motor->current_fdb = mavlink_msg_dji_motor_get_current_fdb(msg);
    dji_motor->current_out = mavlink_msg_dji_motor_get_current_out(msg);
    dji_motor->id = mavlink_msg_dji_motor_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DJI_MOTOR_LEN? msg->len : MAVLINK_MSG_ID_DJI_MOTOR_LEN;
        memset(dji_motor, 0, MAVLINK_MSG_ID_DJI_MOTOR_LEN);
    memcpy(dji_motor, _MAV_PAYLOAD(msg), len);
#endif
}
