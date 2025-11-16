#include "pico/stdlib.h"
#include "mcp251x/mcp251x.h"

int main()
{
    spi_instance_t *spi_device = spi_instance_init(SPI_HW_0, 2, 3, 4, 5, false, 10000000);

    MCP251x *can_device = mcp251x_get_device();

    mcp251x_config info;
    info.model = MODEL_MCP2515;
    info.crystal_oscillator = MCP251x_12MHZ;
    info.spi_dev = spi_device;

    if (mcp251x_init(can_device, &info) != MCP251x_ERR_SUCCESS)
        return -1;

    mcp251x_set_bitrate(can_device, CAN_BITRATE_500KBPS);

    mcp251x_destroy(can_device);
    spi_instance_destroy(spi_device);
    return 0;
}