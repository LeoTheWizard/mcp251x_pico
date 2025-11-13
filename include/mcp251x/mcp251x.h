/**
 * @file mcp251x.h
 * @brief C Library for driving the MicroChip MCP251x family of CAN controllers, specifically for the raspberry pico.
 * @author Leo Walker - leowalk183@gmail.com
 * @date 2025/10/1
 * @see mcp251x.c & can.h & spi.h
 *      Also see README.md for usage.
 * @cite https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP2515-Family-Data-Sheet-DS20001801K.pdf
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "mcp251x/platform/spi.h"
#include "mcp251x/can.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @enum mcp251x_error
     * @brief API error handling return values.
     */
    typedef enum
    {
        MCP251x_ERR_SUCCESS,        // Successful operation.
        MCP251x_ERR_FULL,           // No more messages can be sent as Transmit buffers are all full.
        MCP251x_ERR_EMPTY,          // No more messages to read as Recieve buffers are empty.
        MCP251x_ERR_INVALID,        // Invalid parameters.
        MCP251x_ERR_FAIL,           // Operation failed.
        MCP251x_ERR_NOT_SUPPORTED,  // Operation not supported by the controller model or config.
        MCP251x_ERR_NOT_INITIALISED // Device object passed into function, hasn't been initialised.
    } mcp251x_error;

    /**
     * @enum mcp251x_model
     * @brief The different models in the mcp251x CAN controller family.
     */
    typedef enum
    {
        MODEL_MCP2510,
        MODEL_MCP2515,
        MODEL_MCP25625
    } mcp251x_model;

    /**
     * @enum mcp251x_crystal_frequency
     * @brief Physical oscillator source frequency for the mcp251x chip.
     */
    typedef enum
    {
        MCP251x_8MHZ,
        MCP251x_10MHZ,
        MCP251x_12MHZ,
        MCP251x_16MHZ,
        MCP251x_20MHZ
    } mcp251x_crystal_frequency;

    /**
     * @struct mcp251x_config
     * @brief Configuration values for initialising a mcp251x device.
     */
    typedef struct
    {
        mcp251x_model model;
        spi_instance_t *spi_dev;
        mcp251x_crystal_frequency crystal_oscillator;
    } mcp251x_config;

    /**
     * @brief Represents a mcp251x device.
     * @note Pass this to the functions below to drive the controller.
     */
    typedef struct _mcp251x_device MCP251x;

    /**
     * @brief Get uninitialised MCP251x device context.
     */
    MCP251x mcp251x_get_device();

    /**
     * @brief Create a mcp251x device context and initialize the chip.
     * @param device The mcp251x device to init.
     * @param config The mcp251x configuration structure.
     * @return MCP251x_ERR_OK if successfully initialised.
     *         MCP251x_ERR_INVALID if config values are not valid.
     */
    mcp251x_error mcp251x_init(MCP251x *device, mcp251x_config *config);

    /**
     * @brief Deinitialise the chip & puts it to sleep.
     * Night night...
     * @param device The mcp251x device.
     */
    void mcp251x_deinit(MCP251x *device);

    /**
     * @enum mcp251x_transmit_priority
     * @brief Higher priority messages are transmitted first.
     * @note Use with mcp251x_transmit_frame_priority()
     */
    typedef enum
    {
        MCP251x_PRIORITY_NONE = 0x0,
        MCP251x_PRIORITY_LOW = 0x1,
        MCP251x_PRIORITY_MEDIUM = 0x2,
        MCP251x_PRIORITY_HIGH = 0x3
    } mcp251x_transmit_priority;

    /**
     * @enum mcp251x_operation_mode
     * @brief Represents all operational modes of the mcp251x.
     */
    typedef enum
    {
        MCP251x_MODE_NORMAL = 0x00,
        MCP251x_MODE_SLEEP = 0x20,
        MCP251x_MODE_LOOPBACK = 0x40,
        MCP251x_MODE_LISTENONLY = 0x60,
        MCP251x_MODE_CONFIG = 0x80,
        MCP251x_MODE_POWERUP = 0xE0
    } mcp251x_operation_mode;

    /**
     * @brief Change the operational mode of the mcp251x.
     * @param device The mcp251x device.
     * @param mode The operating mode to change to.
     * @return MCP251x_ERR_SUCCESS if mode was changed.
     */
    mcp251x_error mcp251x_set_mode(MCP251x *device, const mcp251x_operation_mode mode);

    /**
     * @brief Trigger a software reset of the mcp251x and reinitialise parameters.
     * @details Device will wake in config mode.
     * @param device The mcp251x device.
     * @return MCP251x_ERR_SUCCESS if reset successfully.
     */
    mcp251x_error mcp251x_reset(MCP251x *device);

    /**
     * @brief Set the CAN bitrate.
     * @param device The mcp251x device.
     * @param bitrate The required bitrate on the canbus.
     * @warning !! Requires the mcp251x to be in config mode to be successful. !!
     * @return
     */
    mcp251x_error mcp251x_set_bitrate(MCP251x *device, const can_bitrate bitrate);

    typedef enum
    {
        MCP251x_CANINTE_RX0IE = 0b00000001,
        MCP251x_CANINTE_RX1IE = 0b00000010,
        MCP251x_CANINTE_TX0IE = 0b00000100,
        MCP251x_CANINTE_TX1IE = 0b00001000,
        MCP251x_CANINTE_TX2IE = 0b00010000,
    } mcp251x_caninte_bits;

    /**
     * @brief Set interrupt mask on the mcp251x. Controls which flags trigger an interrupt
     * @details See mcp251x_caninte_bits enum for specific flags to enable.
     * @param device The mcp251x device.
     * @param interrupt_mask The interrupt mask to set.
     * @return MCP251x_ERR_SUCCESS if successful.
     */
    mcp251x_error mcp251x_set_interrupts(MCP251x *device, uint8_t interrupt_mask);

    typedef enum
    {
        MCP251x_PRESCALE_1,
        MCP251x_PRESCALE_2,
        MCP251x_PRESCALE_4,
        MCP251x_PRESCALE_8
    } mcp251x_clkout_prescale;

    /**
     * @brief Enable and set the prescaler for the clock out pin. Can be used as clock for other devices.
     * @param device The mcp251x device.
     * @param enabled If the clock out pin should be active.
     * @param prescale The clock prescaler, divides FOSC for output.
     */
    mcp251x_error mcp251x_set_clock_out(MCP251x *device, bool enabled, mcp251x_clkout_prescale prescale);

    /**
     * @brief Set an identifier mask for incoming can frames for either recieve buffer 0 or 1.
     * @param device The mcp251x device.
     * @param mask_num The specific mask to change  (0 - 1).
     * @param id_mask The mask value to set.
     * @note Pair with setting a recieve filter to block certain can_ids from being recieved.
     */
    void mcp251x_set_rx_mask(MCP251x *device, uint8_t mask_num, uint32_t id_mask);

    /**
     * @brief Set a recieve filter for incoming can frames.
     * @param device The mcp251x device.
     * @param filter_num The specific filter (0 - 5).
     * @param id_filter The filter value to set.
     * @note A filter will allow a frame to be accepted if the can_id, post mask, is equal to its value. mask & filter == mask & id.
     */
    void mcp251x_set_rx_filter(MCP251x *device, uint8_t filter_num, uint32_t id_filter);

    /**
     * @brief Set the RXB0 rollover enable flag. Allows frames to rollover into buffer 1 even if buffer 0 is full.
     * @param device The mcp251x device.
     * @param value If it should rollover.
     */
    void mcp251x_set_rx_rollover(MCP251x *device, bool value);

    /**
     * @brief Transmit a frame onto the canbus.
     * @param device The mcp251x device.
     * @param frame A pointer to a can_frame to transmit.
     * @return mcp251x error. If successfully transmitted then MCP251x_ERR_OK. Otherwise check enum.
     */
    mcp251x_error mcp251x_send_frame(MCP251x *device, const can_frame *frame);

    /**
     * @brief Transmit a frame onto the canbus with priority.
     * @param device The mcp251x device.
     * @param frame A pointer to a can_frame to transmit.
     * @param priority The priority of the frame. Higher priority is transmitted first.
     * @return mcp251x error. If successfully transmitted then MCP251x_ERR_OK. Otherwise check enum.
     */
    mcp251x_error mcp251x_send_frame_priority(MCP251x *device, const can_frame *frame, const mcp251x_transmit_priority priority);

    /**
     * @brief Returns number of transmit buffers that are available.
     * @param device The mcp251x device.
     * @return The number of available tx buffers.
     */
    uint8_t mcp251x_get_available_tx_buffer_count(MCP251x *device);

    /**
     * @brief Aborts sending frames from all 3 transmission buffers. Blocks until all buffers are seen to be empty.
     * @param device The mcp251x device.
     * @return MCP251x_ERR_SUCCESS if messages were aborted successfully.
     *         MCP251x_ERR_FAIL if buffers weren't cleared in time.
     */
    mcp251x_error mcp251x_abort_all_transmission(MCP251x *device);

    /**
     * @brief Read a recieved can frame off the canbus.
     * @param device The mcp251x device.
     * @param frame A pointer to a can_frame structure to read the frame into.
     * @return MCP251x_ERROR_OK if successfully recieved a frame.
     */
    mcp251x_error mcp251x_read_frame(MCP251x *device, can_frame *frame);

    /**
     * @brief Read all available frames in the recieve buffers.
     * @param device The mcp251x device.
     * @param frame_buffer A pointer to a can_frame array to read the frames into.
     * @param frame_count Set to how many frames have been read.
     * @note Only two available frame buffers, so array size only needs to be two & frame_count will be 0 - 2.
     * @return mcp251x error. MCP251x_ERROR_RXB_EMPTY if there are no frames.
     */
    mcp251x_error mcp251x_read_all_frames(MCP251x *device, can_frame *frame_buffer, int *frame_count);

    /**
     * @brief Get the status of the transmit and recieve buffers.
     * @ref Look at FIGURE 12-8 in datasheet for flags.
     * @param device The mcp251x device.
     * @return The value of the status.
     */
    uint8_t mcp251x_get_buffers_status(MCP251x *device);

    /**
     * @brief Mark all recieve buffers as empty ready for new frames.
     * @param device The mcp251x device.
     */
    void mcp251x_clear_rx_buffers(MCP251x *device);

    /**
     * @brief Get the current state of the interrupt flags.
     * @ref See CANINTF register in datasheet for flags.
     * @param device The mcp251x device.
     * @return The value of the CANINTF register.
     */
    uint8_t mcp251x_get_interrupts(MCP251x *device);

    /**
     * @brief Clear the mcp251x interrupt flags.
     * @param device The mcp251x device.
     */
    void mcp251x_clear_interrupts(MCP251x *device);

    /**
     * @brief Get the interupt mask which determines which interupt flags trigger the interrupt pin.
     * @ref See CANINTE register in datasheet for flags.
     * @param device The mcp251x device.
     * @return The value of the CANINTE register.
     */
    uint8_t mcp251x_get_interrupt_mask(MCP251x *device);

    /**
     * @brief Get the error flags of the mcp251x.
     * @ref See EFLG register in datasheet for flags.
     * @param device The mcp251x device.
     * @return The value of the EFLG register.
     */
    uint8_t mcp251x_get_error_flags(MCP251x *device);

    /**
     * @brief Get the recieve error count.
     * @details
     * @param device The mcp251x device.
     * @return The error count from the REC register.
     */
    uint8_t mcp251x_get_recieve_error_count(MCP251x *device);

    /**
     * @brief Get the transmit error count.
     * @details
     * @param device The mcp251x device.
     * @return The error count from the TEC register.
     */
    uint8_t mcp251x_get_transmit_error_count(MCP251x *device);

#ifdef __cplusplus
}
#endif