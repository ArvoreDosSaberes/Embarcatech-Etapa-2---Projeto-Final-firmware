#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

// Define the flash target offset. 
// A naive approach is to use a fixed address near the end of the flash, 
// ensuring it doesn't overlap with your firmware code.
// The total flash size for a Pico is typically 2MB (0x200000).
#define FLASH_TARGET_OFFSET (1 * 1024 * 1024) // Example: 1MB offset

// Calculate the start address of the persistent data region in XIP memory
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

void save_data_to_flash(uint32_t data_to_save) {
    // Create a buffer to hold the data for the flash page (256 bytes)
    uint8_t data_buffer[FLASH_PAGE_SIZE];
    // Copy the data into the buffer
    memcpy(data_buffer, &data_to_save, sizeof(data_to_save));

    // Disable interrupts and pause other core if necessary (handled by SDK functions)
    uint32_t ints = save_and_disable_interrupts();
    
    // Erase the 4KB sector containing the target address
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    
    // Program the flash with the data in the buffer
    flash_range_program(FLASH_TARGET_OFFSET, data_buffer, FLASH_PAGE_SIZE);
    
    // Restore interrupts
    restore_interrupts(ints);
}

uint32_t load_data_from_flash() {
    uint32_t loaded_data;
    // Data can be read directly from the XIP memory address
    memcpy(&loaded_data, flash_target_contents, sizeof(loaded_data));
    return loaded_data;
}
