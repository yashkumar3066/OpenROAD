// Vector_Input.v
module Vector_Input #(
    parameter ROWS = 4
)(
    input wire clk,                                   // Clock signal
    input wire [ROWS*8-1:0] vector_in_flat,           // Flattened input activation vector (4*8=32 bits)
    output reg [ROWS*8-1:0] voltages_flat             // Flattened output voltages to crossbar rows
);

    integer i;
    always @(posedge clk) begin
        for (i = 0; i < ROWS; i = i + 1) begin
            voltages_flat[i*8 +: 8] <= vector_in_flat[i*8 +: 8];
        end
    end

endmodule

