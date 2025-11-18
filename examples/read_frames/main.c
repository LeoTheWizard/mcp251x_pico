#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "mcp251x/mcp251x.h"

int main()
{
    stdio_init_all();

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

    // frame object.
    can_frame frame;

    // Read frames on canbus and print them to the serial output.
    while (1)
    {
        if (mcp251x_read_frame(can_device, &frame) == MCP251x_ERR_SUCCESS)
        {
            printf("[ID: 0x%x DLC: %d]", can_frame_get_msg_id(&frame), frame.dlc);
            for (int i = 0; i < frame.dlc; i++)
            {
                printf(" %02x", frame.data[i]);
            }
            printf("\n");
        }
        sleep_ms(1);
    }

    // Clean up memory, ensure can controller is destroyed first.
    mcp251x_destroy(can_device);
    spi_instance_destroy(spi_device);
    return 0;
}