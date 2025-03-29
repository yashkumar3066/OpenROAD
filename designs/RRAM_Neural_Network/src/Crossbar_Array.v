
module Crossbar_Array #(
    parameter ROWS = 4,
    parameter COLS = 4
)(
    input wire [ROWS-1:0] set_signals,                        // Set signals for each row
    input wire [ROWS-1:0] reset_signals,                      // Reset signals for each row
    input wire [ROWS*COLS*8-1:0] resistance_values_flat,       // Flattened resistance values (16*8=128 bits for 4x4)
    output wire [ROWS*COLS*8-1:0] resistance_outs_flat        // Flattened output resistance values
);

    genvar i, j;
    generate
        for (i = 0; i < ROWS; i = i + 1) begin: row_loop
            for (j = 0; j < COLS; j = j + 1) begin: col_loop
                // Calculate the starting bit for the current cell
                localparam START_BIT = (i * COLS + j) * 8;
                
                // Instantiate RRAM_Cell
                RRAM_Cell rram_cell (
                    .set(set_signals[i]),
                    .reset(reset_signals[i]),
                    .resistance_in(resistance_values_flat[START_BIT +: 8]),
                    .resistance_out(resistance_outs_flat[START_BIT +: 8])
                );
            end
        end
    endgenerate

endmodule

