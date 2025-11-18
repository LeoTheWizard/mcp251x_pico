#include <stdlib.h>
#include <string.h>

#include "mcp251x/mcp251x.h"
#include "mcp251x/platform/sleep.h"

#define CANCTRL_REQOP 0xE0
#define CANSTAT_OPMOD 0xE0

#define MCP251x_STATUS_TXREQ_MASK(txb_num) (1 << ((txb_num * 2) + 2)) // Mask for transmit requested bit on status register.

#define MCP251x_BUFFER_SIZE 14

const char *mcp251x_strerror(mcp251x_error error)
{
    if (error > 6)
        return "UNKNOWN";

    static const char *error_strings[] = {
        "Success",
        "Buffer full",
        "Buffer empty",
        "Invalid parameter",
        "Failure",
        "Not supported",
        "Device not initialised"};

    return error_strings[error];
}

typedef enum
{
    MCP251x_SPI_RESET = 0xC0,       //  software reset
    MCP251x_SPI_WRITE = 0x02,       //  write register
    MCP251x_SPI_READ = 0x03,        //  read register
    MCP251x_SPI_RTS = 0x80,         //  request to send
    MCP251x_SPI_READ_STATUS = 0xA0, //  read status register
    MCP251x_SPI_BIT_MODIFY = 0x05,  //  modify register
    MCP251x_SPI_LOAD_TX = 0x40,     //  set register pointer to a point in either 3 of the transmit buffers. for quick access
    MCP251x_SPI_READ_RX = 0x90,     //  set register pointer to a point in either 2 of the recieve buffers.
    MCP251x_SPI_RX_STATUS = 0xB0    //  read rx status.
} mcp251x_spi_cmd;

typedef enum
{
    MCP251x_REG_RXF0SIDH = 0x00, //  Recieve buffer 0 - Standard Identfier Filter High Byte
    MCP251x_REG_RXF0SIDL = 0x01, //  Recieve buffer 0 - Standard Identfier Filter Low Byte
    MCP251x_REG_RXF0EID8 = 0x02,
    MCP251x_REG_RXF0EID0 = 0x03,

    MCP251x_REG_RXF1SIDH = 0x04,
    MCP251x_REG_RXF1SIDL = 0x05,
    MCP251x_REG_RXF1EID8 = 0x06,
    MCP251x_REG_RXF1EID0 = 0x07,

    MCP251x_REG_RXF2SIDH = 0x08,
    MCP251x_REG_RXF2SIDL = 0x09,
    MCP251x_REG_RXF2EID8 = 0x0A,
    MCP251x_REG_RXF2EID0 = 0x0B,

    MCP251x_REG_BFPCTRL = 0x0C, // Pin control & status register.
    MCP251x_REG_TXRTSCTRL = 0x0D,

    MCP251x_REG_CANSTAT = 0x0E, //  CAN Status
    MCP251x_REG_CANCTRL = 0x0F, //  CAN Control

    MCP251x_REG_RXF3SIDH = 0x10,
    MCP251x_REG_RXF3SIDL = 0x11,
    MCP251x_REG_RXF3EID8 = 0x12,
    MCP251x_REG_RXF3EID0 = 0x13,

    MCP251x_REG_RXF4SIDH = 0x14,
    MCP251x_REG_RXF4SIDL = 0x15,
    MCP251x_REG_RXF4EID8 = 0x16,
    MCP251x_REG_RXF4EID0 = 0x17,

    MCP251x_REG_RXF5SIDH = 0x18,
    MCP251x_REG_RXF5SIDL = 0x19,
    MCP251x_REG_RXF5EID8 = 0x1A,
    MCP251x_REG_RXF5EID0 = 0x1B,

    MCP251x_REG_TEC = 0x1C, //  Transmit Error Count
    MCP251x_REG_REC = 0x1D, //  Recieve Error Count

    MCP251x_REG_RXM0SIDH = 0x20, // Recieve buffer 0 - Standard Identifier Mask High Byte
    MCP251x_REG_RXM0SIDL = 0x21, // Recieve buffer 0 - Standard Identifier Mask Low Byte
    MCP251x_REG_RXM0EID8 = 0x22, // Recieve buffer 0 - Extended Identifier Mask High Byte
    MCP251x_REG_RXM0EID0 = 0x23, // Recieve buffer 0 - Extended Identifier Mask Low Byte

    MCP251x_REG_RXM1SIDH = 0x24, // Recieve buffer 1 - Standard Identifier Mask High Byte
    MCP251x_REG_RXM1SIDL = 0x25, // Recieve buffer 1 - Standard Identifier Mask Low Byte
    MCP251x_REG_RXM1EID8 = 0x26, // Recieve buffer 1 - Extended Identifier Mask High Byte
    MCP251x_REG_RXM1EID0 = 0x27, // Recieve buffer 1 - Extended Identifier Mask Low Byte

    MCP251x_REG_CNF3 = 0x28, // Clock Configuration register 3
    MCP251x_REG_CNF2 = 0x29, // Clock Configuration register 2
    MCP251x_REG_CNF1 = 0x2A, // Clock Configuration register 1

    MCP251x_REG_CANINTE = 0x2B, // CAN interupt enable registor
    MCP251x_REG_CANINTF = 0x2C, // CAN interupt flag registor
    MCP251x_REG_EFLG = 0x2D,    // Error Flag registor

    MCP251x_REG_TXB0CTRL = 0x30, // Transmit buffer 0 - Control Register
    MCP251x_REG_TXB0SIDH = 0x31, // Transmit buffer 0 - Stardand ID High byte
    MCP251x_REG_TXB0SIDL = 0x32, // Transmit buffer 0 - Stardard ID Low byte
    MCP251x_REG_TXB0EID8 = 0x33, // Transmit buffer 0 - Extended Identifier High Byte
    MCP251x_REG_TXB0EID0 = 0x34, // Transmit buffer 0 - Extended Identifier Low Byte
    MCP251x_REG_TXB0DLC = 0x35,  // Transmit buffer 0 - Data length
    MCP251x_REG_TXB0DATA = 0x36, // Transmit buffer 0 - First of Data bytes 0x36 - 0x3D

    MCP251x_REG_TXB1CTRL = 0x40, // Transmit buffer 1 - Control Register
    MCP251x_REG_TXB1SIDH = 0x41, // Transmit buffer 1 - Stardand ID High byte
    MCP251x_REG_TXB1SIDL = 0x42, // Transmit buffer 1 - Stardard ID Low byte
    MCP251x_REG_TXB1EID8 = 0x43, // Transmit buffer 1 - Extended Identifier High Byte
    MCP251x_REG_TXB1EID0 = 0x44, // Transmit buffer 1 - Extended Identifier Low Byte
    MCP251x_REG_TXB1DLC = 0x45,  // Transmit buffer 1 - Data length
    MCP251x_REG_TXB1DATA = 0x46, // Transmit buffer 1 - First of Data bytes 0x46 - 0x4D

    MCP251x_REG_TXB2CTRL = 0x50, // Transmit buffer 2 - Control Register
    MCP251x_REG_TXB2SIDH = 0x51, // Transmit buffer 2 - Stardand ID High byte
    MCP251x_REG_TXB2SIDL = 0x52, // Transmit buffer 2 - Stardard ID Low byte
    MCP251x_REG_TXB2EID8 = 0x53, // Transmit buffer 2 - Extended Identifier High Byte
    MCP251x_REG_TXB2EID0 = 0x54, // Transmit buffer 2 - Extended Identifier Low Byte
    MCP251x_REG_TXB2DLC = 0x55,  // Transmit buffer 2 - Data length
    MCP251x_REG_TXB2DATA = 0x56, // Transmit buffer 2 - First of Data bytes 0x56 - 0x5D

    MCP251x_REG_RXB0CTRL = 0x60, // Recieve buffer 0 - Control Register
    MCP251x_REG_RXB0SIDH = 0x61, // Recieve buffer 0 - Stardand ID High byte
    MCP251x_REG_RXB0SIDL = 0x62, // Recieve buffer 0 - Stardard ID Low byte
    MCP251x_REG_RXB0EID8 = 0x63, // Recieve buffer 0 - Extended Identifier High Byte
    MCP251x_REG_RXB0EID0 = 0x64, // Recieve buffer 0 - Extended Identifier Low Byte
    MCP251x_REG_RXB0DLC = 0x65,  // Recieve buffer 0 - Data length
    MCP251x_REG_RXB0DATA = 0x66, // Recieve buffer 0 - First of Data bytes 0x66 - 0x6D

    MCP251x_REG_RXB1CTRL = 0x70, // Recieve buffer 1 - Control Register
    MCP251x_REG_RXB1SIDH = 0x71, // Recieve buffer 1 - Stardand ID High byte
    MCP251x_REG_RXB1SIDL = 0x72, // Recieve buffer 1 - Stardard ID Low byte
    MCP251x_REG_RXB1EID8 = 0x73, // Recieve buffer 1 - Extended Identifier High Byte
    MCP251x_REG_RXB1EID0 = 0x74, // Recieve buffer 1 - Extended Identifier Low Byte
    MCP251x_REG_RXB1DLC = 0x75,  // Recieve buffer 1 - Data length
    MCP251x_REG_RXB1DATA = 0x76  // Recieve buffer 1 - First of Data bytes 0x76 - 0x7D
} mcp251x_register;

enum mcp251x_canintf_bits
{
    MCP251x_CANINTF_RX0IF = 0b00000001,
    MCP251x_CANINTF_RX1IF = 0b00000010,
    MCP251x_CANINTF_TX0IF = 0b00000100,
    MCP251x_CANINTF_TX1IF = 0b00001000,
    MCP251x_CANINTF_TX2IF = 0b00010000,
    MCP251x_CANINTF_ERRIF = 0b00100000,
    MCP251x_CANINTF_WAKIF = 0b01000000,
    MCP251x_CANINTF_MERRF = 0b10000000
};

enum mcp251x_canctrl_bits
{
    MCP251x_CANCTRL_CLKPRE0 = 0b00000001, // Clock out prescaler bit 0.
    MCP251x_CANCTRL_CLKPRE1 = 0b00000010, // Clock out prescaler bit 1.
    MCP251x_CANCTRL_CLKEN = 0b00000100,   // Clock out enable bit.
    MCP251x_CANCTRL_OSM = 0b00001000,     // One-shot mode bit.
    MCP251x_CANCTRL_ABAT = 0b00010000,    // Abort all pending transmission bit.
    MCP251x_CANCTRL_REQOP0 = 0b00100000,  // Request operational mode bit 0.
    MCP251x_CANCTRL_REQOP1 = 0b01000000,  // Request operational mode bit 1.
    MCP251x_CANCTRL_REQOP2 = 0b10000000,  // Request operational mode bit 2.
};

enum mcp251x_error_flag_bits
{
    MCP251x_EFLG_EWARN = 0b00000001,
    MCP251x_EFLG_RXWAR = 0b00000010,
    MCP251x_EFLG_TXWAR = 0b00000100,
    MCP251x_EFLG_RXEP = 0b00001000,
    MCP251x_EFLG_TXEP = 0b00010000,
    MCP251x_EFLG_TXBO = 0b00100000,
    MCP251x_EFLG_RX0OVR = 0b01000000,
    MCP251x_EFLG_RX1OVR = 0b10000000
};

enum mcp251x_txbn_ctrl_bits
{
    MCP251x_TXB_ABTF = 0b01000000,
    MCP251x_TXB_MLOA = 0b00100000,
    MCP251x_TXB_TXERR = 0b00010000,
    MCP251x_TXB_TXREQ = 0b00001000,
    MCP251x_TXB_TXIE = 0b00000100,
    MCP251x_TXB_TXP = 0b00000011
};

enum mcp251x_buffer_layout
{
    MCP251x_BUFFER_SIDH,
    MCP251x_BUFFER_SIDL,
    MCP251x_BUFFER_EID8,
    MCP251x_BUFFER_EID0,
    MCP251x_BUFFER_DLC,
    MCP251x_BUFFER_DATA
};

/**
 * @brief Set a range of contigious registers. Provided a starting register and a length.
 * @ref See mcp251x_registers enum.
 * @param dev The spi device context.
 * @param start_reg The starting register.
 * @param read_buffer Buffer to set the register values.
 * @param length The length in bytes.
 */
static void mcp251x_set_registers(spi_instance_t *dev, const mcp251x_register start_reg, const uint8_t *write_buffer, uint32_t length)
{
    uint8_t buffer[2] = {MCP251x_SPI_WRITE, start_reg};
    spi_instance_chip_enable(dev);
    spi_instance_transmit(dev, buffer, 2);
    spi_instance_transmit(dev, write_buffer, length);
    spi_instance_chip_disable(dev);
}

/**
 * @brief Set the value of a single register.
 * @ref See mcp251x_registers enum.
 * @param dev The spi device context.
 * @param reg The register to set.
 * @param data The value to set the register.
 */
static void mcp251x_set_register(spi_instance_t *dev, const mcp251x_register reg, uint8_t data)
{
    uint8_t buffer[3] = {MCP251x_SPI_WRITE, reg, data};
    spi_instance_chip_enable(dev);
    spi_instance_transmit(dev, buffer, 3);
    spi_instance_chip_disable(dev);
}

/**
 * @brief Read an array of contigious registers. Provided a starting register and a length.
 * @ref See mcp251x_registers enum.
 * @param dev The spi device context.
 * @param start_reg The starting register.
 * @param read_buffer Buffer to store the register values.
 * @param length The length in bytes.
 */
static void mcp251x_read_registers(spi_instance_t *dev, const mcp251x_register start_reg, uint8_t *read_buffer, uint32_t length)
{
    uint8_t buffer[2] = {MCP251x_SPI_READ, start_reg};
    spi_instance_chip_enable(dev);
    spi_instance_transmit(dev, buffer, 2);

    spi_instance_recieve(dev, read_buffer, length);
    spi_instance_chip_disable(dev);
}

/**
 * @brief Read a single register.
 * @ref See mcp251x_registers enum.
 * @param dev The spi device context.
 * @param reg The register to read.
 * @return The value of the register.
 */
static uint8_t mcp251x_read_register(spi_instance_t *dev, const mcp251x_register reg)
{
    uint8_t buffer[3] = {MCP251x_SPI_READ, reg, 0x00};
    spi_instance_chip_enable(dev);
    spi_instance_transfer(dev, buffer, buffer, 3);
    spi_instance_chip_disable(dev);
    return buffer[2];
}

/**
 * @brief Modify bits of a register using a mask and value.
 * @ref See mcp251x_registers enum.
 * @param dev The spi device context.
 * @param reg The register to modify.
 * @param mask The bits you would like to set.
 * @param data The values to set.
 */
static void mcp251x_modify_register(spi_instance_t *dev, const mcp251x_register reg, const uint8_t mask, const uint8_t data)
{
    uint8_t buffer[4] = {MCP251x_SPI_BIT_MODIFY, reg, mask, data};
    spi_instance_chip_enable(dev);
    spi_instance_transmit(dev, buffer, 4);
    spi_instance_chip_disable(dev);
}

/**
 * @brief Get the status of the transmit and recieve buffers from the mcp251x over SPI.
 * @ref Look at FIGURE 12-8 in datasheet for flags.
 * @param dev The spi device context.
 * @return The value of the status.
 */
static uint8_t mcp251x_read_status(spi_instance_t *dev)
{
    uint8_t buffer[2] = {MCP251x_SPI_READ_STATUS, 0x00};
    spi_instance_chip_enable(dev);
    spi_instance_transfer(dev, buffer, buffer, 2);
    spi_instance_chip_disable(dev);
    return buffer[1];
}

/**
 * @brief Read the status of the recieve buffers.
 * @ref Look at FIGURE 12-9 in datasheet for flags.
 * @param dev The spi device context.
 * @return The value of the status.
 */
static uint8_t mcp251x_read_rx_status(spi_instance_t *dev)
{
    uint8_t buffer[2] = {MCP251x_SPI_RX_STATUS, 0x00};
    spi_instance_chip_enable(dev);
    spi_instance_transfer(dev, buffer, buffer, 2);
    spi_instance_chip_disable(dev);
    return buffer[1];
}

/**
 * @brief Mark a transmit buffer as ready to send.
 * @param dev The spi device context.
 * @param tx_buffer_num The transmit buffer number to send.
 */
static void mcp251x_request_to_send(spi_instance_t *dev, const uint8_t tx_buffer_num)
{
    spi_instance_chip_enable(dev);
    spi_instance_transmit_byte(dev, MCP251x_SPI_RTS | (1 << tx_buffer_num));
    spi_instance_chip_disable(dev);
}

/**
 * @brief Load a transmit buffer with data.
 * @note This is more efficient than setting the respective registers as it uses less spi instructions.
 * @param dev The spi device context.
 * @param tx_buffer_num The transmit buffer number to load.
 * @param transmit_buffer The data to load into the transmit buffer.
 * @warning Ensure buffer is atleast 5 + frame length in size, otherwise an overflow will occur.
 */
static void mcp251x_load_tx_buffer(spi_instance_t *dev, const uint8_t tx_buffer_num, const uint8_t *transmit_buffer)
{
    uint8_t cmd = MCP251x_SPI_LOAD_TX | (tx_buffer_num << 1);
    spi_instance_chip_enable(dev);
    spi_instance_transmit_byte(dev, cmd);
    spi_instance_transmit(dev, transmit_buffer, 5 + transmit_buffer[MCP251x_BUFFER_DLC]);
    spi_instance_chip_disable(dev);
}

/**
 * @brief Read a recieve buffer.
 * @note This is more efficient than reading the respective registers as it uses less spi instructions and will automatically clear the buffer once the CS has gone back high.
 * @param dev The spi device context.
 * @param rxb_num The recieve buffer number to read.
 * @param buffer_data A buffer to populate with recieved message.
 * @warning Ensure buffer_data is atleast 13 bytes to accomodate for the largest frames (DLC == 8).
 */
static void mcp251x_read_rx_buffer(spi_instance_t *dev, uint8_t rxb_num, uint8_t *buffer_data)
{
    uint8_t cmd = MCP251x_SPI_READ_RX | (rxb_num << 2);
    spi_instance_chip_enable(dev);
    spi_instance_transmit_byte(dev, cmd);

    spi_instance_recieve(dev, buffer_data, 5);
    if ((buffer_data[MCP251x_BUFFER_DLC] & 0xF) < CAN_MAX_DATA_LENGTH)
        spi_instance_recieve(dev, buffer_data + 5, buffer_data[MCP251x_BUFFER_DLC] & 0xF);
    else
        spi_instance_recieve(dev, buffer_data + 5, CAN_MAX_DATA_LENGTH);

    spi_instance_chip_disable(dev);
}

struct _mcp251x_device
{
    mcp251x_config config;
    mcp251x_operation_mode current_mode;
    bool initialised;
};

MCP251x *mcp251x_get_device()
{
    MCP251x *dev = malloc(sizeof(MCP251x));
    memset(dev, 0, sizeof(MCP251x)); // zero initialise.

    return dev;
}

mcp251x_error mcp251x_init(MCP251x *device, mcp251x_config *config)
{
    // Validate config.
    if (config->model > 2)
        return MCP251x_ERR_INVALID;
    if (config->crystal_oscillator > 4)
        return MCP251x_ERR_INVALID;

    // Copy config to parameters.
    device->config = *config;
    device->initialised = true;
    device->current_mode = MCP251x_MODE_CONFIG;
    return MCP251x_ERR_SUCCESS;
};

void mcp251x_destroy(MCP251x *device)
{
    // Place the chip into a low power mode.
    if (device->initialised)
        mcp251x_set_mode(device, MCP251x_MODE_SLEEP);

    free(device);
}

mcp251x_error mcp251x_reset(MCP251x *device)
{
    if (!device->initialised)
        return MCP251x_ERR_NOT_INITIALISED;

    spi_instance_chip_enable(device->config.spi_dev);
    spi_instance_transmit_byte(device->config.spi_dev, MCP251x_SPI_RESET);
    spi_instance_chip_disable(device->config.spi_dev);

    mcp251x_sleep_us(100);

    uint8_t zeros[MCP251x_BUFFER_SIZE];
    memset(zeros, 0, sizeof(zeros));

    mcp251x_set_registers(device->config.spi_dev, MCP251x_REG_TXB0CTRL, zeros, MCP251x_BUFFER_SIZE);
    mcp251x_set_registers(device->config.spi_dev, MCP251x_REG_TXB1CTRL, zeros, MCP251x_BUFFER_SIZE);
    mcp251x_set_registers(device->config.spi_dev, MCP251x_REG_TXB2CTRL, zeros, MCP251x_BUFFER_SIZE); // Clear Transmit buffers
    mcp251x_set_register(device->config.spi_dev, MCP251x_REG_CANINTF, 0);                            // Clear interrupt flags

    mcp251x_set_register(device->config.spi_dev, MCP251x_REG_RXB0CTRL, 0x04);
    mcp251x_set_register(device->config.spi_dev, MCP251x_REG_RXB1CTRL, 0x00);

    return MCP251x_ERR_SUCCESS;
}

mcp251x_error mcp251x_set_mode(MCP251x *device, const mcp251x_operation_mode mode)
{
    if (!device->initialised)
        return MCP251x_ERR_NOT_INITIALISED; // Device context is not initialised.

    if (device->current_mode == mode)
        return MCP251x_ERR_SUCCESS; // Already in the requested mode.

    // Request operating mode.
    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_CANCTRL, CANCTRL_REQOP, mode);

    // Wait for change to be made.
    for (uint8_t i = 0; i < 10; ++i)
    {
        uint8_t newmode = mcp251x_read_register(device->config.spi_dev, MCP251x_REG_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        if (newmode == mode)
        {
            device->current_mode = mode;
            return MCP251x_ERR_SUCCESS; // Operating mode has changed.
        }

        mcp251x_sleep_us(100);
    }

    // Timed out, mode didn't change.
    return MCP251x_ERR_FAIL;
}

typedef struct
{
    uint8_t cnf3;
    uint8_t cnf2;
    uint8_t cnf1;
} mcp251x_clock_cfg;

// TODO: Finish the clock conmfigurations.
static const mcp251x_clock_cfg s_mcp251x_clockconfigs_8mhz[CAN_BITRATE_MAX] = {
    {0x80, 0x80, 0x00}, // 1MBPS
    {0, 0, 0},          // 800KBPS
    {0x82, 0x90, 0x00}, // 500KBPS
    {0x85, 0xB1, 0x00}, // 250KBPS
    {0x86, 0xB4, 0x00}, // 200KBPS
    {0x85, 0xB1, 0x01}, // 125KBPS
    {0x86, 0xB4, 0x01}, // 100KBPS
    {0x87, 0xBF, 0x01}, // 80KBPS
    {0x86, 0xB4, 0x03}, // 50KBPS
    {0x87, 0xBF, 0x03}, // 40KBPS
    {0x87, 0xBF, 0x07}, // 20KBPS
    {0x87, 0xBF, 0x0F}, // 10KBPS
    {0x87, 0xBF, 0x1F}  // 5KBPS
};

static const mcp251x_clock_cfg s_mcp251x_clockconfigs_10mhz[CAN_BITRATE_MAX] = {
    {0x80, 0xC8, 0x00}, // 1MBPS
    {0, 0, 0},          // 800KBPS
    {0x82, 0xD9, 0x40}, // 500KBPS
    {0x82, 0xD9, 0x41}, // 250KBPS
    {0, 0, 0},          // 200KBPS
    {0x82, 0xD9, 0x43}, // 125KBPS
    {0, 0, 0},          // 100KBPS
    {0, 0, 0},          // 80KBPS
    {0, 0, 0},          // 50KBPS
    {0, 0, 0},          // 40KBPS
    {0, 0, 0},          // 20KBPS
    {0, 0, 0},          // 10KBPS
    {0, 0, 0}           // 5KBPS
};

static const mcp251x_clock_cfg s_mcp251x_clockconfigs_12mhz[CAN_BITRATE_MAX] = {
    {0x80, 0x82, 0x00}, // 1MBPS
    {0, 0, 0},          // 800KBPS
    {0x84, 0x84, 0x00}, // 500KBPS
    {0, 0, 0},          // 250KBPS
    {0x84, 0xF1, 0x41}, // 200KBPS * Used Calculator
    {0, 0, 0},          // 125KBPS
    {0, 0, 0},          // 100KBPS
    {0, 0, 0},          // 80KBPS
    {0, 0, 0},          // 50KBPS
    {0, 0, 0},          // 40KBPS
    {0, 0, 0},          // 20KBPS
    {0, 0, 0},          // 10KBPS
    {0, 0, 0}           // 5KBPS
};

static const mcp251x_clock_cfg s_mcp251x_clockconfigs_16mhz[CAN_BITRATE_MAX] = {
    {0x82, 0xD0, 0x00}, // 1MBPS
    {0, 0, 0},          // 800KBPS
    {0x86, 0xF0, 0x00}, // 500KBPS
    {0x85, 0xF1, 0x41}, // 250KBPS
    {0x87, 0xFA, 0x01}, // 200KBPS
    {0x86, 0xF0, 0x03}, // 125KBPS
    {0x87, 0xFA, 0x03}, // 100KBPS
    {0x87, 0xFF, 0x03}, // 80KBPS
    {0x87, 0xFA, 0x07}, // 50KBPS
    {0x87, 0xFF, 0x07}, // 40KBPS
    {0x87, 0xFF, 0x0F}, // 20KBPS
    {0x87, 0xFF, 0x1F}, // 10KBPS
    {0x87, 0xFF, 0x3F}  // 5KBPS
};

static const mcp251x_clock_cfg s_mcp251x_clockconfigs_20mhz[CAN_BITRATE_MAX] = {
    {0x82, 0xD9, 0x00}, // 1MBPS
    {0, 0, 0},          // 800KBPS
    {0x87, 0xFA, 0x00}, // 500KBPS
    {0x86, 0xFB, 0x41}, // 250KBPS
    {0x87, 0xFF, 0x01}, // 200KBPS
    {0x87, 0xFA, 0x03}, // 125KBPS
    {0x87, 0xFA, 0x04}, // 100KBPS
    {0x87, 0xFF, 0x04}, // 80KBPS
    {0x87, 0xFA, 0x09}, // 50KBPS
    {0x87, 0xFF, 0x09}, // 40KBPS
    {0, 0, 0},          // 20KBPS
    {0, 0, 0},          // 10KBPS
    {0, 0, 0}           // 5KBPS
};

static const mcp251x_clock_cfg *s_mcp251x_clock_config_map[] = {
    s_mcp251x_clockconfigs_8mhz,
    s_mcp251x_clockconfigs_10mhz,
    s_mcp251x_clockconfigs_12mhz,
    s_mcp251x_clockconfigs_16mhz,
    s_mcp251x_clockconfigs_20mhz};

mcp251x_error mcp251x_set_bitrate(MCP251x *device, const can_bitrate bitrate)
{
    if (!device->initialised)
        return MCP251x_ERR_NOT_INITIALISED;

    // Ensure config mode.
    if (device->current_mode != MCP251x_MODE_CONFIG)
        mcp251x_set_mode(device, MCP251x_MODE_CONFIG);

    // Look up clock configuration values from map.
    mcp251x_clock_cfg cfg = s_mcp251x_clock_config_map[device->config.crystal_oscillator][bitrate];

    if (cfg.cnf1 == 0 && cfg.cnf2 == 0 && cfg.cnf3 == 0)
        return MCP251x_ERR_NOT_SUPPORTED; // Oscillator & Bitrate configuration not supported.

    mcp251x_set_registers(device->config.spi_dev, MCP251x_REG_CNF3, (uint8_t *)&cfg, 3);
    return MCP251x_ERR_SUCCESS;
}

mcp251x_error mcp251x_set_interrupts(MCP251x *device, uint8_t interrupt_mask)
{
    if (!device->initialised)
        return MCP251x_ERR_NOT_INITIALISED;

    mcp251x_set_register(device->config.spi_dev, MCP251x_REG_CANINTE, interrupt_mask);
    return MCP251x_ERR_SUCCESS;
}

mcp251x_error mcp251x_set_clock_out(MCP251x *device, bool enabled, mcp251x_clkout_prescale prescale)
{
    if (prescale > 3)
        return MCP251x_ERR_INVALID;

    uint8_t value = (enabled << 2) | prescale;
    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_CANCTRL, 0x0F, value);
    return MCP251x_ERR_SUCCESS;
}

/**
 * @brief Converts a can id into the id register format in the mcp251x buffers.
 * @param buffer_data The byte array to be copied to a buffer's identifier regs.
 * @param id The can id to convert, use CAN_EFF_FLAG and CAN_EXT_FLAG for extended and remote frame ids.
 */
static void mcp251x_id_to_buffer(uint8_t *buffer_data, uint32_t id)
{
    uint16_t can_id = (uint16_t)(id & 0x0FFFF);
    const bool ext = id & CAN_EFF_FLAG;
    id = (id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));
    if (ext)
    {
        buffer_data[MCP251x_BUFFER_EID0] = (uint8_t)(can_id & 0xFF);
        buffer_data[MCP251x_BUFFER_EID8] = (uint8_t)(can_id >> 8);
        can_id = (uint16_t)(id >> 16);
        buffer_data[MCP251x_BUFFER_SIDL] = (uint8_t)(can_id & 0x03);
        buffer_data[MCP251x_BUFFER_SIDL] += (uint8_t)((can_id & 0x1C) << 3);
        buffer_data[MCP251x_BUFFER_SIDL] |= 8;
        buffer_data[MCP251x_BUFFER_SIDH] = (uint8_t)(can_id >> 5);
    }
    else
    {
        buffer_data[MCP251x_BUFFER_SIDH] = (uint8_t)(can_id >> 3);
        buffer_data[MCP251x_BUFFER_SIDL] = (uint8_t)((can_id & 0x07) << 5);
        buffer_data[MCP251x_BUFFER_EID0] = 0;
        buffer_data[MCP251x_BUFFER_EID8] = 0;
    }
}

#define MCP251x_SIDL_EFF_BIT 0x08

/**
 * @brief Converts an identifier buffer into a can id.
 * @param buffer The id buffer from the registers.
 * @param ctrl The buffer control register.
 * @return The can id.
 */
static uint32_t mcp251x_buffer_to_id(const uint8_t *buffer, uint8_t ctrl)
{
    // Standard Identifier
    uint32_t can_id = (buffer[MCP251x_BUFFER_SIDH] << 3) + (buffer[MCP251x_BUFFER_SIDL] >> 5);

    if (buffer[MCP251x_BUFFER_SIDL] & MCP251x_SIDL_EFF_BIT) // Extended Identifier
    {
        can_id = (can_id << 2) + (buffer[MCP251x_BUFFER_SIDL] & 0x03);
        can_id = (can_id << 8) + buffer[MCP251x_BUFFER_EID8];
        can_id = (can_id << 8) + buffer[MCP251x_BUFFER_EID0];
        can_id |= CAN_EFF_FLAG; // Sets MSB CAN_EEF_FLAG
    }
    if (ctrl & 0x08)
        can_id |= CAN_RTR_FLAG;

    return can_id;
}

mcp251x_error mcp251x_set_rx_mask(MCP251x *device, uint8_t mask_num, uint32_t id_mask)
{
    if (mask_num > 1)
        return MCP251x_ERR_INVALID;

    // Ensure config mode.
    if (device->current_mode != MCP251x_MODE_CONFIG)
        mcp251x_set_mode(device, MCP251x_MODE_CONFIG);

    mcp251x_register reg_addr = MCP251x_REG_RXM0EID0 + (mask_num * 0x04);

    uint8_t id_buffer[4];
    mcp251x_id_to_buffer(id_buffer, id_mask);

    mcp251x_set_registers(device->config.spi_dev, reg_addr, id_buffer, 4);

    return MCP251x_ERR_SUCCESS;
}

mcp251x_error mcp251x_set_rx_filter(MCP251x *device, uint8_t filter_num, uint32_t id_filter)
{
    if (filter_num > 5) // Invalid filter number.
        return MCP251x_ERR_INVALID;

    // Ensure config mode.
    if (device->current_mode != MCP251x_MODE_CONFIG)
        mcp251x_set_mode(device, MCP251x_MODE_CONFIG);

    // Register map
    static const mcp251x_register filter_regs[6] = {
        MCP251x_REG_RXF0SIDH,
        MCP251x_REG_RXF1SIDH,
        MCP251x_REG_RXF2SIDH,
        MCP251x_REG_RXF3SIDH,
        MCP251x_REG_RXF4SIDH,
        MCP251x_REG_RXF5SIDH,
    };

    // Convert filter id into the mcp251x buffer format.
    uint8_t id_buffer[4];
    mcp251x_id_to_buffer(id_buffer, id_filter);

    // Write filter to registers.
    mcp251x_set_registers(device->config.spi_dev, filter_regs[filter_num], id_buffer, 4);

    return MCP251x_ERR_SUCCESS;
}

void mcp251x_set_rx_rollover(MCP251x *device, bool value)
{
    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_RXB0CTRL, 0x04, value << 2);
}

enum mcp251x_bfpctrl_bits
{
    MCP251x_BFPCTRL_B0BFM = 0b00000001,
    MCP251x_BFPCTRL_B1BFM = 0b00000010,
    MCP251x_BFPCTRL_B0BFE = 0b00000100,
    MCP251x_BFPCTRL_B1BFE = 0b00001000,
    MCP251x_BFPCTRL_B0BFS = 0b00010000,
    MCP251x_BFPCTRL_B1BFS = 0b00100000
};

mcp251x_error mcp251x_pin_control(MCP251x *device, uint8_t pin_number, mcp251x_pin_mode mode)
{
    if (pin_number > 1) // Invalid pin number.
        return MCP251x_ERR_INVALID;

    switch (mode)
    {
    case MCP251x_PIN_DISABLED:
        mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_BFPCTRL, MCP251x_BFPCTRL_B0BFE << pin_number, 0x00);
        break;

    case MCP251x_PIN_BF_INTERRUPT:
    {
        uint8_t mask = (MCP251x_BFPCTRL_B0BFE | MCP251x_BFPCTRL_B0BFM) << pin_number;
        mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_BFPCTRL, mask, mask);
        break;
    }
    case MCP251x_PIN_DIGITAL_OUT:
    {
        // Enable pin, set operation mode to digital output, clear pin value to low.
        uint8_t mask = (MCP251x_BFPCTRL_B0BFE | MCP251x_BFPCTRL_B0BFM | MCP251x_BFPCTRL_B0BFS) << pin_number;
        mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_BFPCTRL, mask, MCP251x_BFPCTRL_B0BFE << pin_number);
        break;
    }
    }
    return MCP251x_ERR_SUCCESS;
}

mcp251x_error mcp251x_set_pin(MCP251x *device, uint8_t pin_number, bool state)
{
    if (pin_number > 1) // Invalid pin number.
        return MCP251x_ERR_INVALID;

    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_BFPCTRL, MCP251x_BFPCTRL_B0BFS << pin_number, (MCP251x_BFPCTRL_B0BFS << pin_number) * state);
    return MCP251x_ERR_SUCCESS;
}

static mcp251x_error mcp251x_get_available_tx_buffer(MCP251x *device, uint8_t *txbn)
{
    *txbn = -1;
    uint8_t status = mcp251x_read_status(device->config.spi_dev);
    for (int i = 0; i < 3; i++)
    {
        if ((status & MCP251x_STATUS_TXREQ_MASK(i)) == 0)
        {
            *txbn = i;
            break;
        }
    }
    if (*txbn == -1)
        return MCP251x_ERR_FULL;

    return MCP251x_ERR_SUCCESS;
}

static mcp251x_error mcp251x_send_frame_on_buf(MCP251x *device, const can_frame *frame, uint8_t txbn)
{
    uint8_t frame_buffer[MCP251x_BUFFER_SIZE - 1];

    mcp251x_id_to_buffer(frame_buffer, frame->id);

    bool rtr = (frame->id & CAN_RTR_FLAG);
    frame_buffer[MCP251x_BUFFER_DLC] = rtr ? (frame->dlc | 64) : frame->dlc; // Set rtr bit

    memcpy(frame_buffer + MCP251x_BUFFER_DATA, frame->data, frame->dlc);

    // Populate transmit buffer & Request transmit
    mcp251x_load_tx_buffer(device->config.spi_dev, txbn, frame_buffer);
    mcp251x_request_to_send(device->config.spi_dev, txbn);

    uint8_t ctrl = mcp251x_read_register(device->config.spi_dev, MCP251x_REG_TXB0CTRL + (txbn * 16));
    if ((ctrl & (MCP251x_TXB_ABTF | MCP251x_TXB_MLOA | MCP251x_TXB_TXERR)) != 0)
        return MCP251x_ERR_FAIL;

    return MCP251x_ERR_SUCCESS;
}

mcp251x_error mcp251x_send_frame(MCP251x *device, const can_frame *frame)
{
    if (frame->dlc > CAN_MAX_DATA_LENGTH)
        return MCP251x_ERR_INVALID; // Checks data length is within CAN_MAX_DATA_LENGTH = 8

    // Find available TX buffer
    uint8_t TXBn = 0;
    mcp251x_error err = mcp251x_get_available_tx_buffer(device, &TXBn);
    if (err != MCP251x_ERR_SUCCESS)
        return err;

    // Send frame.
    return mcp251x_send_frame_on_buf(device, frame, TXBn);
}

mcp251x_error mcp251x_send_frame_priority(MCP251x *device, const can_frame *frame, const mcp251x_transmit_priority priority)
{
    if (frame->dlc > CAN_MAX_DATA_LENGTH)
        return MCP251x_ERR_INVALID; // Checks data length is within CAN_MAX_DATA_LENGTH = 8

    // Find available TX buffer
    uint8_t TXBn = 0;
    mcp251x_error err = mcp251x_get_available_tx_buffer(device, &TXBn);
    if (err != MCP251x_ERR_SUCCESS)
        return err;

    // Set priority.
    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_TXB0CTRL + (TXBn * 16), 0x03, priority);

    // Send frame.
    return mcp251x_send_frame_on_buf(device, frame, TXBn);
}

static void mcp251x_read_frame_buffer(MCP251x *device, int rxbn, can_frame *frame)
{
    uint8_t ctrl;
    uint8_t frame_buffer[MCP251x_BUFFER_SIZE - 1];

    ctrl = mcp251x_read_register(device->config.spi_dev, MCP251x_REG_RXB0CTRL + (rxbn * 16));
    mcp251x_read_rx_buffer(device->config.spi_dev, rxbn, frame_buffer);

    uint32_t id = mcp251x_buffer_to_id(frame_buffer, ctrl);

    frame->id = id;
    frame->dlc = frame_buffer[MCP251x_BUFFER_DLC] & 0xF;
    memcpy(frame->data, frame_buffer + MCP251x_BUFFER_DATA, frame->dlc);
}

mcp251x_error mcp251x_read_frame(MCP251x *device, can_frame *frame)
{
    uint8_t stat = mcp251x_read_status(device->config.spi_dev);

    //  Find a full buffer
    if (stat & 0x01)
        mcp251x_read_frame_buffer(device, 0, frame);
    else if (stat & 0x02)
        mcp251x_read_frame_buffer(device, 1, frame);
    else
        return MCP251x_ERR_EMPTY; // No buffers ready

    return MCP251x_ERR_SUCCESS;
}

mcp251x_error mcp251x_read_all_frames(MCP251x *device,
                                      can_frame *frame_buffer,
                                      int *frame_count)
{
    uint8_t stat = mcp251x_read_status(device->config.spi_dev);
    int frame_index = 0;

    if (stat & 0x01)
    {
        mcp251x_read_frame_buffer(device, 0, frame_buffer + frame_index);
        frame_index++;
    }

    if (stat & 0x02)
    {
        mcp251x_read_frame_buffer(device, 1, frame_buffer + frame_index);
        frame_index++;
    }

    *frame_count = frame_index;

    if (frame_index == 0)
        return MCP251x_ERR_EMPTY;
    else
        return MCP251x_ERR_SUCCESS;
}

void mcp251x_clear_rx_buffers(MCP251x *device)
{
    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_CANINTF, 3, 0);
}

uint8_t mcp251x_get_available_tx_buffer_count(MCP251x *device)
{
    uint8_t stat = mcp251x_read_status(device->config.spi_dev);
    uint8_t count = 0;
    for (int i = 0; i < 3; ++i)
    {
        if ((stat & MCP251x_STATUS_TXREQ_MASK(i)) == 0)
            count++;
    }

    return count;
}

mcp251x_error mcp251x_abort_all_transmission(MCP251x *device)
{
    mcp251x_error err = MCP251x_ERR_FAIL;
    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_CANCTRL, MCP251x_CANCTRL_ABAT, MCP251x_CANCTRL_ABAT);

    uint8_t mask = MCP251x_STATUS_TXREQ_MASK(0) | MCP251x_STATUS_TXREQ_MASK(1) | MCP251x_STATUS_TXREQ_MASK(2);
    for (uint8_t i = 0; i < 10; ++i)
    {
        uint8_t stat = mcp251x_read_status(device->config.spi_dev);
        if ((stat & mask) == 0)
        {
            err = MCP251x_ERR_SUCCESS;
            break;
        }
        mcp251x_sleep_us(100);
    }

    mcp251x_modify_register(device->config.spi_dev, MCP251x_REG_CANCTRL, MCP251x_CANCTRL_ABAT, 0);
    return err;
}

uint8_t mcp251x_get_buffers_status(MCP251x *device)
{
    return mcp251x_read_status(device->config.spi_dev);
}

uint8_t mcp251x_get_interrupts(MCP251x *device)
{
    return mcp251x_read_register(device->config.spi_dev, MCP251x_REG_CANINTF);
}

void mcp251x_clear_interrupts(MCP251x *device)
{
    mcp251x_set_register(device->config.spi_dev, MCP251x_REG_CANINTF, 0);
}

uint8_t mcp251x_get_interrupt_mask(MCP251x *device)
{
    return mcp251x_read_register(device->config.spi_dev, MCP251x_REG_CANINTE);
}

uint8_t mcp251x_get_error_flags(MCP251x *device)
{
    return mcp251x_read_register(device->config.spi_dev, MCP251x_REG_EFLG);
}

uint8_t mcp251x_get_recieve_error_count(MCP251x *device)
{
    return mcp251x_read_register(device->config.spi_dev, MCP251x_REG_REC);
}

uint8_t mcp251x_get_transmit_error_count(MCP251x *device)
{
    return mcp251x_read_register(device->config.spi_dev, MCP251x_REG_TEC);
}