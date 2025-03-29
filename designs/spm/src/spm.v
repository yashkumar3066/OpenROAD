module spm (
    input wire clk,
    input wire rst,
    input wire start,
    input wire serial_mult,
    input wire [7:0] parallel_mult,
    output reg [15:0] product,
    output reg done
);
    reg [7:0] shift_reg;
    reg [3:0] count;
    reg active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            product <= 16'd0;
            shift_reg <= 8'd0;
            count <= 4'd0;
            active <= 1'b0;
            done <= 1'b0;
        end else if (start) begin
            shift_reg <= parallel_mult;
            product <= 16'd0;
            count <= 4'd0;
            active <= 1'b1;
            done <= 1'b0;
        end else if (active) begin
            if (serial_mult) begin
                product[15:8] <= product[15:8] + shift_reg;
            end
            product <= product >> 1;
            count <= count + 1;
            
            if (count == 4'd8) begin
                active <= 1'b0;
                done <= 1'b1;
            end
        end
    end

endmodule

