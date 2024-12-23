// RamLog.h
#pragma once

namespace ram {

/**
 * @enum RamLogMessageID
 * @brief Enumeration of unique message IDs for logging.
 */
enum RamLogMessageID {
    RAM_INIT_SUCCESS = 1,          // Initialization successful
    RAM_INIT_FAILURE = 2,          // Initialization failed
    RAM_CREATE_NET_FAILURE = 3,    // Failed to create a net
    RAM_CREATE_BTERM_FAILURE = 4,  // Failed to create a BTerm
    RAM_CREATE_INSTANCE_FAILURE = 5, // Failed to create an instance
    RAM_MISSING_STORAGE_CELL = 6,   // Storage cell not found
    RAM_MISSING_TRISTATE_CELL = 7,  // Tristate cell not found
    RAM_MISSING_INV_CELL = 8,        // Inverter cell not found
    RAM_MISSING_AND_GATE = 9,        // AND gate cell not found
    RAM_MISSING_CLOCK_GATE = 10,     // Clock gate cell not found
    RAM_CREATED_NET = 11,            // Net created successfully
    RAM_CREATED_BTERM = 12,          // BTerm created successfully
    RAM_CREATED_INSTANCE = 13,       // Instance created successfully
    RAM_CREATED_BIT = 14,            // Bit created successfully
    RAM_CREATED_BYTE = 15,           // Byte created successfully
    RAM_DECODER_LOGIC_ERROR = 16,    // Decoder logic error
    RAM_SELECTION_ERROR = 17,        // Selection error
    RAM_GENERATION_COMPLETE = 18     // RAM generation completed
    // Add more as needed
};

} // namespace ram

