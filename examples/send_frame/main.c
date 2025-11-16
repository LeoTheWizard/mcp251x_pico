#include "pico/stdlib.h"
#include <string.h>
#include "mcp251x/mcp251x.h"

int main()
{
    // Create an SPI instance for comms between MCU & MCP251x
    spi_instance_t *spi_device = spi_instance_init(SPI_HW_1, 10, 11, 12, 13, false, 10000000);

    // Allocate device memory.
    MCP251x *can_device = mcp251x_get_device();

    mcp251x_config info;
    info.model = MODEL_MCP2515;
    info.crystal_oscillator = MCP251x_12MHZ;
    info.spi_dev = spi_device;

    // Initialise the chip with config.
    if (mcp251x_init(can_device, &info) != MCP251x_ERR_SUCCESS)
        return -1;

    // Change bitrate to 500 KBPS
    mcp251x_set_bitrate(can_device, CAN_BITRATE_500KBPS);

    // After configuration always remember to change to normal in order to use the canbus.
    mcp251x_set_mode(can_device, MCP251x_MODE_NORMAL);

    // Build frame object.
    can_frame frame;
    frame.id = 0x77;
    frame.dlc = 8;
    uint64_t data = 0xEFBEADDE;
    memcpy(frame.data, (void *)&data, 8);

    // Send frame onto canbus
    mcp251x_send_frame(can_device, &frame);

    // Blocking wait, replace with work or other can commands.
    while (1)
    {
        sleep_ms(1000);
    }

    // Clean up memory, ensure can controller is destroyed first.
    mcp251x_destroy(can_device);
    spi_instance_destroy(spi_device);
    return 0;
}