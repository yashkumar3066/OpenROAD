
module RRAM_Cell (
    input wire set,                    // Control signal to set resistance
    input wire reset,                  // Control signal to reset resistance
    input wire [7:0] resistance_in,    // Input resistance value (8-bit)
    output reg [7:0] resistance_out    // Output resistance value (8-bit)
);

    always @(posedge set or posedge reset) begin
        if (set)
            resistance_out <= resistance_in;
        else if (reset)
            resistance_out <= 8'd0; // Reset to default resistance (0 Ohms)
    end

endmodule

