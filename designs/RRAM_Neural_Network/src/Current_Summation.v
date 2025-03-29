
module Current_Summation #(
    parameter ROWS = 4,
    parameter COLS = 4
)(
    input wire [ROWS*8-1:0] voltages_flat,                     // Input voltages from Vector_Input (32 bits)
    input wire [ROWS*COLS*8-1:0] resistances_flat,            // Flattened resistance values from Crossbar_Array (128 bits)
    output wire [COLS*16-1:0] output_vector_flat               // Flattened output vector after summation (4*16=64 bits)
);

    genvar i, j;
    generate
        for (j = 0; j < COLS; j = j + 1) begin: col_loop
            wire [15:0] current [0:ROWS-1]; // Current from each row in this column

            for (i = 0; i < ROWS; i = i + 1) begin: row_loop
                // Calculate the starting bit for the current cell
                localparam RES_START_BIT = (i * COLS + j) * 8;

                // Calculate the starting bit for the output vector
                localparam OUT_START_BIT = j * 16;

                // Simple I = V/R with scaling (V is 8 bits, R is 8 bits)
                // To prevent division by zero, check if resistance is not zero
                assign current[i] = (resistances_flat[RES_START_BIT +: 8] != 8'd0) ?
                                    (voltages_flat[i*8 +: 8] * 16'd100) / resistances_flat[RES_START_BIT +: 8] :
                                    16'd0;
            end

            // Sum all currents for this column
            assign output_vector_flat[j*16 +: 16] = current[0] + current[1] + current[2] + current[3];
        end
    endgenerate

endmodule

