module RAM8x8 (clock,
    A,
    Din,
    Dout,
    write_enable);
 input clock;
 input [2:0] A;
 input [7:0] Din;
 input [7:0] Dout;
 input [0:0] write_enable;

 wire \address_buffer.buffer_mid_0 ;
 wire \address_buffer.buffer_mid_1 ;
 wire \address_buffer.buffer_mid_2 ;
 wire \decoder.A0_b ;
 wire \decoder.A1_b ;
 wire \decoder.A2_b ;
 wire \decoder.out0_mid ;
 wire \decoder.out1_mid ;
 wire \decoder.out2_mid ;
 wire \decoder.out3_mid ;
 wire \decoder.out4_mid ;
 wire \decoder.out5_mid ;
 wire \decoder.out6_mid ;
 wire \decoder.out7_mid ;
 wire \input_buffer.buffer_mid_0 ;
 wire \input_buffer.buffer_mid_1 ;
 wire \input_buffer.buffer_mid_2 ;
 wire \input_buffer.buffer_mid_3 ;
 wire \input_buffer.buffer_mid_4 ;
 wire \input_buffer.buffer_mid_5 ;
 wire \input_buffer.buffer_mid_6 ;
 wire \input_buffer.buffer_mid_7 ;
 wire \ram.Do_internal_0_0 ;
 wire \ram.Do_internal_0_1 ;
 wire \ram.Do_internal_0_2 ;
 wire \ram.Do_internal_0_3 ;
 wire \ram.Do_internal_0_4 ;
 wire \ram.Do_internal_0_5 ;
 wire \ram.Do_internal_0_6 ;
 wire \ram.Do_internal_0_7 ;
 wire \storage_0_0.we_net ;
 wire \storage_0_0.clock_b ;
 wire \storage_0_0.gclock ;
 wire \storage_0_0.select0_b ;
 wire \storage_0_0.bit0.storage ;
 wire \storage_0_0.bit1.storage ;
 wire \storage_0_0.bit2.storage ;
 wire \storage_0_0.bit3.storage ;
 wire \storage_0_0.bit4.storage ;
 wire \storage_0_0.bit5.storage ;
 wire \storage_0_0.bit6.storage ;
 wire \storage_0_0.bit7.storage ;
 wire \storage_1_0.we_net ;
 wire \storage_1_0.clock_b ;
 wire \storage_1_0.gclock ;
 wire \storage_1_0.select0_b ;
 wire \storage_1_0.bit0.storage ;
 wire \storage_1_0.bit1.storage ;
 wire \storage_1_0.bit2.storage ;
 wire \storage_1_0.bit3.storage ;
 wire \storage_1_0.bit4.storage ;
 wire \storage_1_0.bit5.storage ;
 wire \storage_1_0.bit6.storage ;
 wire \storage_1_0.bit7.storage ;
 wire \storage_2_0.we_net ;
 wire \storage_2_0.clock_b ;
 wire \storage_2_0.gclock ;
 wire \storage_2_0.select0_b ;
 wire \storage_2_0.bit0.storage ;
 wire \storage_2_0.bit1.storage ;
 wire \storage_2_0.bit2.storage ;
 wire \storage_2_0.bit3.storage ;
 wire \storage_2_0.bit4.storage ;
 wire \storage_2_0.bit5.storage ;
 wire \storage_2_0.bit6.storage ;
 wire \storage_2_0.bit7.storage ;
 wire \storage_3_0.we_net ;
 wire \storage_3_0.clock_b ;
 wire \storage_3_0.gclock ;
 wire \storage_3_0.select0_b ;
 wire \storage_3_0.bit0.storage ;
 wire \storage_3_0.bit1.storage ;
 wire \storage_3_0.bit2.storage ;
 wire \storage_3_0.bit3.storage ;
 wire \storage_3_0.bit4.storage ;
 wire \storage_3_0.bit5.storage ;
 wire \storage_3_0.bit6.storage ;
 wire \storage_3_0.bit7.storage ;
 wire \storage_4_0.we_net ;
 wire \storage_4_0.clock_b ;
 wire \storage_4_0.gclock ;
 wire \storage_4_0.select0_b ;
 wire \storage_4_0.bit0.storage ;
 wire \storage_4_0.bit1.storage ;
 wire \storage_4_0.bit2.storage ;
 wire \storage_4_0.bit3.storage ;
 wire \storage_4_0.bit4.storage ;
 wire \storage_4_0.bit5.storage ;
 wire \storage_4_0.bit6.storage ;
 wire \storage_4_0.bit7.storage ;
 wire \storage_5_0.we_net ;
 wire \storage_5_0.clock_b ;
 wire \storage_5_0.gclock ;
 wire \storage_5_0.select0_b ;
 wire \storage_5_0.bit0.storage ;
 wire \storage_5_0.bit1.storage ;
 wire \storage_5_0.bit2.storage ;
 wire \storage_5_0.bit3.storage ;
 wire \storage_5_0.bit4.storage ;
 wire \storage_5_0.bit5.storage ;
 wire \storage_5_0.bit6.storage ;
 wire \storage_5_0.bit7.storage ;
 wire \storage_6_0.we_net ;
 wire \storage_6_0.clock_b ;
 wire \storage_6_0.gclock ;
 wire \storage_6_0.select0_b ;
 wire \storage_6_0.bit0.storage ;
 wire \storage_6_0.bit1.storage ;
 wire \storage_6_0.bit2.storage ;
 wire \storage_6_0.bit3.storage ;
 wire \storage_6_0.bit4.storage ;
 wire \storage_6_0.bit5.storage ;
 wire \storage_6_0.bit6.storage ;
 wire \storage_6_0.bit7.storage ;
 wire \storage_7_0.we_net ;
 wire \storage_7_0.clock_b ;
 wire \storage_7_0.gclock ;
 wire \storage_7_0.select0_b ;
 wire \storage_7_0.bit0.storage ;
 wire \storage_7_0.bit1.storage ;
 wire \storage_7_0.bit2.storage ;
 wire \storage_7_0.bit3.storage ;
 wire \storage_7_0.bit4.storage ;
 wire \storage_7_0.bit5.storage ;
 wire \storage_7_0.bit6.storage ;
 wire \storage_7_0.bit7.storage ;
 wire \output_buffer.buffer_mid_0 ;
 wire \output_buffer.buffer_mid_1 ;
 wire \output_buffer.buffer_mid_2 ;
 wire \output_buffer.buffer_mid_3 ;
 wire \output_buffer.buffer_mid_4 ;
 wire \output_buffer.buffer_mid_5 ;
 wire \output_buffer.buffer_mid_6 ;
 wire \output_buffer.buffer_mid_7 ;
 wire [7:0] \decoder.word_select ;
 wire [7:0] \ram.Di0 ;
 wire [2:0] \ram.addr ;

 sky130_fd_sc_hd__inv_1 \address_buffer.inv1_0  (.A(A[0]),
    .Y(\address_buffer.buffer_mid_0 ));
 sky130_fd_sc_hd__inv_1 \address_buffer.inv2_0  (.A(\address_buffer.buffer_mid_0 ),
    .Y(\ram.addr [0]));
 sky130_fd_sc_hd__inv_1 \address_buffer.inv1_1  (.A(A[1]),
    .Y(\address_buffer.buffer_mid_1 ));
 sky130_fd_sc_hd__inv_1 \address_buffer.inv2_1  (.A(\address_buffer.buffer_mid_1 ),
    .Y(\ram.addr [1]));
 sky130_fd_sc_hd__inv_1 \address_buffer.inv1_2  (.A(A[2]),
    .Y(\address_buffer.buffer_mid_2 ));
 sky130_fd_sc_hd__inv_1 \address_buffer.inv2_2  (.A(\address_buffer.buffer_mid_2 ),
    .Y(\ram.addr [2]));
 sky130_fd_sc_hd__inv_1 \decoder.invA0  (.A(\ram.addr [0]),
    .Y(\decoder.A0_b ));
 sky130_fd_sc_hd__inv_1 \decoder.invA1  (.A(\ram.addr [1]),
    .Y(\decoder.A1_b ));
 sky130_fd_sc_hd__inv_1 \decoder.invA2  (.A(\ram.addr [2]),
    .Y(\decoder.A2_b ));
 sky130_fd_sc_hd__and2_0 \decoder.out0_and1  (.A(\decoder.A2_b ),
    .B(\decoder.A1_b ),
    .X(\decoder.out0_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out0_and2  (.A(\decoder.out0_mid ),
    .B(\decoder.A0_b ),
    .X(\decoder.word_select [0]));
 sky130_fd_sc_hd__and2_0 \decoder.out1_and1  (.A(\decoder.A2_b ),
    .B(\decoder.A1_b ),
    .X(\decoder.out1_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out1_and2  (.A(\decoder.out1_mid ),
    .B(\ram.addr [0]),
    .X(\decoder.word_select [1]));
 sky130_fd_sc_hd__and2_0 \decoder.out2_and1  (.A(\decoder.A2_b ),
    .B(\ram.addr [1]),
    .X(\decoder.out2_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out2_and2  (.A(\decoder.out2_mid ),
    .B(\decoder.A0_b ),
    .X(\decoder.word_select [2]));
 sky130_fd_sc_hd__and2_0 \decoder.out3_and1  (.A(\decoder.A2_b ),
    .B(\ram.addr [1]),
    .X(\decoder.out3_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out3_and2  (.A(\decoder.out3_mid ),
    .B(\ram.addr [0]),
    .X(\decoder.word_select [3]));
 sky130_fd_sc_hd__and2_0 \decoder.out4_and1  (.A(\ram.addr [2]),
    .B(\decoder.A1_b ),
    .X(\decoder.out4_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out4_and2  (.A(\decoder.out4_mid ),
    .B(\decoder.A0_b ),
    .X(\decoder.word_select [4]));
 sky130_fd_sc_hd__and2_0 \decoder.out5_and1  (.A(\ram.addr [2]),
    .B(\decoder.A1_b ),
    .X(\decoder.out5_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out5_and2  (.A(\decoder.out5_mid ),
    .B(\ram.addr [0]),
    .X(\decoder.word_select [5]));
 sky130_fd_sc_hd__and2_0 \decoder.out6_and1  (.A(\ram.addr [2]),
    .B(\ram.addr [1]),
    .X(\decoder.out6_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out6_and2  (.A(\decoder.out6_mid ),
    .B(\decoder.A0_b ),
    .X(\decoder.word_select [6]));
 sky130_fd_sc_hd__and2_0 \decoder.out7_and1  (.A(\ram.addr [2]),
    .B(\ram.addr [1]),
    .X(\decoder.out7_mid ));
 sky130_fd_sc_hd__and2_0 \decoder.out7_and2  (.A(\decoder.out7_mid ),
    .B(\ram.addr [0]),
    .X(\decoder.word_select [7]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_0  (.A(Din[0]),
    .Y(\input_buffer.buffer_mid_0 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_0  (.A(\input_buffer.buffer_mid_0 ),
    .Y(\ram.Di0 [0]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_1  (.A(Din[1]),
    .Y(\input_buffer.buffer_mid_1 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_1  (.A(\input_buffer.buffer_mid_1 ),
    .Y(\ram.Di0 [1]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_2  (.A(Din[2]),
    .Y(\input_buffer.buffer_mid_2 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_2  (.A(\input_buffer.buffer_mid_2 ),
    .Y(\ram.Di0 [2]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_3  (.A(Din[3]),
    .Y(\input_buffer.buffer_mid_3 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_3  (.A(\input_buffer.buffer_mid_3 ),
    .Y(\ram.Di0 [3]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_4  (.A(Din[4]),
    .Y(\input_buffer.buffer_mid_4 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_4  (.A(\input_buffer.buffer_mid_4 ),
    .Y(\ram.Di0 [4]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_5  (.A(Din[5]),
    .Y(\input_buffer.buffer_mid_5 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_5  (.A(\input_buffer.buffer_mid_5 ),
    .Y(\ram.Di0 [5]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_6  (.A(Din[6]),
    .Y(\input_buffer.buffer_mid_6 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_6  (.A(\input_buffer.buffer_mid_6 ),
    .Y(\ram.Di0 [6]));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv1_7  (.A(Din[7]),
    .Y(\input_buffer.buffer_mid_7 ));
 sky130_fd_sc_hd__inv_1 \input_buffer.inv2_7  (.A(\input_buffer.buffer_mid_7 ),
    .Y(\ram.Di0 [7]));
 sky130_fd_sc_hd__and2_0 \storage_0_0.we_and  (.A(\decoder.word_select [0]),
    .B(write_enable[0]),
    .X(\storage_0_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_0_0.clock_inv  (.A(clock),
    .Y(\storage_0_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_0_0.icg  (.GATE(\storage_0_0.we_net ),
    .GCLK(\storage_0_0.gclock ),
    .CLK(\storage_0_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_0_0.select_inv_0  (.A(\decoder.word_select [0]),
    .Y(\storage_0_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_0_0.bit0.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit0.obuf0  (.A(\storage_0_0.bit0.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_0_0.bit1.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit1.obuf0  (.A(\storage_0_0.bit1.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_0_0.bit2.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit2.obuf0  (.A(\storage_0_0.bit2.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_0_0.bit3.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit3.obuf0  (.A(\storage_0_0.bit3.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_0_0.bit4.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit4.obuf0  (.A(\storage_0_0.bit4.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_0_0.bit5.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit5.obuf0  (.A(\storage_0_0.bit5.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_0_0.bit6.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit6.obuf0  (.A(\storage_0_0.bit6.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_0_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_0_0.bit7.storage ),
    .GATE(\storage_0_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_0_0.bit7.obuf0  (.A(\storage_0_0.bit7.storage ),
    .TE_B(\storage_0_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__and2_0 \storage_1_0.we_and  (.A(\decoder.word_select [1]),
    .B(write_enable[0]),
    .X(\storage_1_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_1_0.clock_inv  (.A(clock),
    .Y(\storage_1_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_1_0.icg  (.GATE(\storage_1_0.we_net ),
    .GCLK(\storage_1_0.gclock ),
    .CLK(\storage_1_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_1_0.select_inv_0  (.A(\decoder.word_select [1]),
    .Y(\storage_1_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_1_0.bit0.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit0.obuf0  (.A(\storage_1_0.bit0.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_1_0.bit1.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit1.obuf0  (.A(\storage_1_0.bit1.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_1_0.bit2.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit2.obuf0  (.A(\storage_1_0.bit2.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_1_0.bit3.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit3.obuf0  (.A(\storage_1_0.bit3.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_1_0.bit4.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit4.obuf0  (.A(\storage_1_0.bit4.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_1_0.bit5.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit5.obuf0  (.A(\storage_1_0.bit5.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_1_0.bit6.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit6.obuf0  (.A(\storage_1_0.bit6.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_1_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_1_0.bit7.storage ),
    .GATE(\storage_1_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_1_0.bit7.obuf0  (.A(\storage_1_0.bit7.storage ),
    .TE_B(\storage_1_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__and2_0 \storage_2_0.we_and  (.A(\decoder.word_select [2]),
    .B(write_enable[0]),
    .X(\storage_2_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_2_0.clock_inv  (.A(clock),
    .Y(\storage_2_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_2_0.icg  (.GATE(\storage_2_0.we_net ),
    .GCLK(\storage_2_0.gclock ),
    .CLK(\storage_2_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_2_0.select_inv_0  (.A(\decoder.word_select [2]),
    .Y(\storage_2_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_2_0.bit0.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit0.obuf0  (.A(\storage_2_0.bit0.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_2_0.bit1.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit1.obuf0  (.A(\storage_2_0.bit1.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_2_0.bit2.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit2.obuf0  (.A(\storage_2_0.bit2.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_2_0.bit3.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit3.obuf0  (.A(\storage_2_0.bit3.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_2_0.bit4.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit4.obuf0  (.A(\storage_2_0.bit4.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_2_0.bit5.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit5.obuf0  (.A(\storage_2_0.bit5.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_2_0.bit6.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit6.obuf0  (.A(\storage_2_0.bit6.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_2_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_2_0.bit7.storage ),
    .GATE(\storage_2_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_2_0.bit7.obuf0  (.A(\storage_2_0.bit7.storage ),
    .TE_B(\storage_2_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__and2_0 \storage_3_0.we_and  (.A(\decoder.word_select [3]),
    .B(write_enable[0]),
    .X(\storage_3_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_3_0.clock_inv  (.A(clock),
    .Y(\storage_3_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_3_0.icg  (.GATE(\storage_3_0.we_net ),
    .GCLK(\storage_3_0.gclock ),
    .CLK(\storage_3_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_3_0.select_inv_0  (.A(\decoder.word_select [3]),
    .Y(\storage_3_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_3_0.bit0.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit0.obuf0  (.A(\storage_3_0.bit0.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_3_0.bit1.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit1.obuf0  (.A(\storage_3_0.bit1.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_3_0.bit2.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit2.obuf0  (.A(\storage_3_0.bit2.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_3_0.bit3.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit3.obuf0  (.A(\storage_3_0.bit3.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_3_0.bit4.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit4.obuf0  (.A(\storage_3_0.bit4.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_3_0.bit5.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit5.obuf0  (.A(\storage_3_0.bit5.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_3_0.bit6.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit6.obuf0  (.A(\storage_3_0.bit6.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_3_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_3_0.bit7.storage ),
    .GATE(\storage_3_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_3_0.bit7.obuf0  (.A(\storage_3_0.bit7.storage ),
    .TE_B(\storage_3_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__and2_0 \storage_4_0.we_and  (.A(\decoder.word_select [4]),
    .B(write_enable[0]),
    .X(\storage_4_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_4_0.clock_inv  (.A(clock),
    .Y(\storage_4_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_4_0.icg  (.GATE(\storage_4_0.we_net ),
    .GCLK(\storage_4_0.gclock ),
    .CLK(\storage_4_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_4_0.select_inv_0  (.A(\decoder.word_select [4]),
    .Y(\storage_4_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_4_0.bit0.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit0.obuf0  (.A(\storage_4_0.bit0.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_4_0.bit1.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit1.obuf0  (.A(\storage_4_0.bit1.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_4_0.bit2.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit2.obuf0  (.A(\storage_4_0.bit2.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_4_0.bit3.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit3.obuf0  (.A(\storage_4_0.bit3.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_4_0.bit4.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit4.obuf0  (.A(\storage_4_0.bit4.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_4_0.bit5.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit5.obuf0  (.A(\storage_4_0.bit5.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_4_0.bit6.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit6.obuf0  (.A(\storage_4_0.bit6.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_4_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_4_0.bit7.storage ),
    .GATE(\storage_4_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_4_0.bit7.obuf0  (.A(\storage_4_0.bit7.storage ),
    .TE_B(\storage_4_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__and2_0 \storage_5_0.we_and  (.A(\decoder.word_select [5]),
    .B(write_enable[0]),
    .X(\storage_5_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_5_0.clock_inv  (.A(clock),
    .Y(\storage_5_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_5_0.icg  (.GATE(\storage_5_0.we_net ),
    .GCLK(\storage_5_0.gclock ),
    .CLK(\storage_5_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_5_0.select_inv_0  (.A(\decoder.word_select [5]),
    .Y(\storage_5_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_5_0.bit0.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit0.obuf0  (.A(\storage_5_0.bit0.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_5_0.bit1.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit1.obuf0  (.A(\storage_5_0.bit1.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_5_0.bit2.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit2.obuf0  (.A(\storage_5_0.bit2.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_5_0.bit3.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit3.obuf0  (.A(\storage_5_0.bit3.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_5_0.bit4.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit4.obuf0  (.A(\storage_5_0.bit4.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_5_0.bit5.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit5.obuf0  (.A(\storage_5_0.bit5.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_5_0.bit6.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit6.obuf0  (.A(\storage_5_0.bit6.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_5_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_5_0.bit7.storage ),
    .GATE(\storage_5_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_5_0.bit7.obuf0  (.A(\storage_5_0.bit7.storage ),
    .TE_B(\storage_5_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__and2_0 \storage_6_0.we_and  (.A(\decoder.word_select [6]),
    .B(write_enable[0]),
    .X(\storage_6_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_6_0.clock_inv  (.A(clock),
    .Y(\storage_6_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_6_0.icg  (.GATE(\storage_6_0.we_net ),
    .GCLK(\storage_6_0.gclock ),
    .CLK(\storage_6_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_6_0.select_inv_0  (.A(\decoder.word_select [6]),
    .Y(\storage_6_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_6_0.bit0.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit0.obuf0  (.A(\storage_6_0.bit0.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_6_0.bit1.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit1.obuf0  (.A(\storage_6_0.bit1.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_6_0.bit2.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit2.obuf0  (.A(\storage_6_0.bit2.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_6_0.bit3.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit3.obuf0  (.A(\storage_6_0.bit3.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_6_0.bit4.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit4.obuf0  (.A(\storage_6_0.bit4.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_6_0.bit5.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit5.obuf0  (.A(\storage_6_0.bit5.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_6_0.bit6.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit6.obuf0  (.A(\storage_6_0.bit6.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_6_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_6_0.bit7.storage ),
    .GATE(\storage_6_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_6_0.bit7.obuf0  (.A(\storage_6_0.bit7.storage ),
    .TE_B(\storage_6_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__and2_0 \storage_7_0.we_and  (.A(\decoder.word_select [7]),
    .B(write_enable[0]),
    .X(\storage_7_0.we_net ));
 sky130_fd_sc_hd__inv_1 \storage_7_0.clock_inv  (.A(clock),
    .Y(\storage_7_0.clock_b ));
 sky130_fd_sc_hd__dlclkp_1 \storage_7_0.icg  (.GATE(\storage_7_0.we_net ),
    .GCLK(\storage_7_0.gclock ),
    .CLK(\storage_7_0.clock_b ));
 sky130_fd_sc_hd__inv_1 \storage_7_0.select_inv_0  (.A(\decoder.word_select [7]),
    .Y(\storage_7_0.select0_b ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit0.bit  (.D(\ram.Di0 [0]),
    .Q(\storage_7_0.bit0.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit0.obuf0  (.A(\storage_7_0.bit0.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_0 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit1.bit  (.D(\ram.Di0 [1]),
    .Q(\storage_7_0.bit1.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit1.obuf0  (.A(\storage_7_0.bit1.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_1 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit2.bit  (.D(\ram.Di0 [2]),
    .Q(\storage_7_0.bit2.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit2.obuf0  (.A(\storage_7_0.bit2.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_2 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit3.bit  (.D(\ram.Di0 [3]),
    .Q(\storage_7_0.bit3.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit3.obuf0  (.A(\storage_7_0.bit3.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_3 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit4.bit  (.D(\ram.Di0 [4]),
    .Q(\storage_7_0.bit4.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit4.obuf0  (.A(\storage_7_0.bit4.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_4 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit5.bit  (.D(\ram.Di0 [5]),
    .Q(\storage_7_0.bit5.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit5.obuf0  (.A(\storage_7_0.bit5.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_5 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit6.bit  (.D(\ram.Di0 [6]),
    .Q(\storage_7_0.bit6.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit6.obuf0  (.A(\storage_7_0.bit6.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_6 ));
 sky130_fd_sc_hd__dlxtp_1 \storage_7_0.bit7.bit  (.D(\ram.Di0 [7]),
    .Q(\storage_7_0.bit7.storage ),
    .GATE(\storage_7_0.gclock ));
 sky130_fd_sc_hd__ebufn_2 \storage_7_0.bit7.obuf0  (.A(\storage_7_0.bit7.storage ),
    .TE_B(\storage_7_0.select0_b ),
    .Z(\ram.Do_internal_0_7 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_0  (.A(\ram.Do_internal_0_0 ),
    .Y(\output_buffer.buffer_mid_0 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_0  (.A(\output_buffer.buffer_mid_0 ),
    .Y(Dout[0]));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_1  (.A(\ram.Do_internal_0_1 ),
    .Y(\output_buffer.buffer_mid_1 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_1  (.A(\output_buffer.buffer_mid_1 ),
    .Y(Dout[1]));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_2  (.A(\ram.Do_internal_0_2 ),
    .Y(\output_buffer.buffer_mid_2 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_2  (.A(\output_buffer.buffer_mid_2 ),
    .Y(Dout[2]));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_3  (.A(\ram.Do_internal_0_3 ),
    .Y(\output_buffer.buffer_mid_3 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_3  (.A(\output_buffer.buffer_mid_3 ),
    .Y(Dout[3]));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_4  (.A(\ram.Do_internal_0_4 ),
    .Y(\output_buffer.buffer_mid_4 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_4  (.A(\output_buffer.buffer_mid_4 ),
    .Y(Dout[4]));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_5  (.A(\ram.Do_internal_0_5 ),
    .Y(\output_buffer.buffer_mid_5 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_5  (.A(\output_buffer.buffer_mid_5 ),
    .Y(Dout[5]));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_6  (.A(\ram.Do_internal_0_6 ),
    .Y(\output_buffer.buffer_mid_6 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_6  (.A(\output_buffer.buffer_mid_6 ),
    .Y(Dout[6]));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv1_7  (.A(\ram.Do_internal_0_7 ),
    .Y(\output_buffer.buffer_mid_7 ));
 sky130_fd_sc_hd__inv_1 \output_buffer.inv2_7  (.A(\output_buffer.buffer_mid_7 ),
    .Y(Dout[7]));
endmodule
