/**
 * @file can.h
 * @brief CAN specific definitions. can_frame definition.
 * @author Leo Walker
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CAN_EFF_FLAG 0x80000000UL /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000UL /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000UL /* error message frame */

#define CAN_SFF_MASK 0x000007FFUL /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFUL /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFUL /* EFF, RTR, ERR flags */

#define CAN_MAX_DATA_LENGTH 8

typedef enum
{
    CAN_BITRATE_1000KBPS,
    CAN_BITRATE_800KBPS,
    CAN_BITRATE_500KBPS,
    CAN_BITRATE_250KBPS,
    CAN_BITRATE_200KBPS,
    CAN_BITRATE_125KBPS,
    CAN_BITRATE_100KBPs,
    CAN_BITRATE_80KBPS,
    CAN_BITRATE_50KBPS,
    CAN_BITRATE_40KBPS,
    CAN_BITRATE_20KBPS,
    CAN_BITRATE_10KBPS,
    CAN_BITRATE_5KBPS,
    CAN_BITRATE_MAX
} can_bitrate;

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[CAN_MAX_DATA_LENGTH];
} can_frame;

static inline uint32_t can_frame_get_msg_id(const can_frame *frame)
{
    return frame->id & CAN_EFF_MASK;
};

static inline bool can_frame_is_ext_id(const can_frame *frame)
{
    return frame->id & CAN_EFF_FLAG;
}

static inline bool can_frame_is_rtr(const can_frame *frame)
{
    return frame->id & CAN_RTR_FLAG;
}

static inline bool can_frame_is_error(const can_frame *frame)
{
    return frame->id & CAN_ERR_FLAG;
}