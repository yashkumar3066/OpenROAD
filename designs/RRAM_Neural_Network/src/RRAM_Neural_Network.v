

module RRAM_Neural_Network #(
    parameter ROWS = 4,
    parameter COLS = 4
)(
    input wire clk,                                                    // Clock signal
    input wire [ROWS-1:0] set_signals,                                // Set signals for Crossbar_Array
    input wire [ROWS-1:0] reset_signals,                              // Reset signals for Crossbar_Array
    input wire [ROWS*COLS*8-1:0] resistance_values_flat,               // Flattened resistance values for Crossbar_Array
    input wire [ROWS*8-1:0] vector_in_flat,                           // Flattened input activation vector
    output wire [COLS*16-1:0] output_vector_flat                      // Flattened output vector after multiplication
);

    wire [ROWS*COLS*8-1:0] resistance_outs_flat; // Output resistances from Crossbar_Array
    wire [ROWS*8-1:0] voltages_flat;            // Voltages from Vector_Input

    // Instantiate Crossbar_Array
    Crossbar_Array #(
        .ROWS(ROWS),
        .COLS(COLS)
    ) crossbar (
        .set_signals(set_signals),
        .reset_signals(reset_signals),
        .resistance_values_flat(resistance_values_flat),
        .resistance_outs_flat(resistance_outs_flat)
    );

    // Instantiate Vector_Input
    Vector_Input #(
        .ROWS(ROWS)
    ) vector_input (
        .clk(clk),
        .vector_in_flat(vector_in_flat),
        .voltages_flat(voltages_flat)
    );

    // Instantiate Current_Summation
    Current_Summation #(
        .ROWS(ROWS),
        .COLS(COLS)
    ) current_sum (
        .voltages_flat(voltages_flat),
        .resistances_flat(resistance_outs_flat),
        .output_vector_flat(output_vector_flat)
    );

endmodule

