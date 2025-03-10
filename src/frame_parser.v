module frame_parser(
    input         clk,          // Clock hệ thống
    input         rst_n,        // Reset active low
    input         valid_frame,  // Cờ báo rằng frame đã được thu thập đầy đủ
    input  [23:0] frame,        // Frame 3 byte đầu vào (24-bit)
    output reg    valid_data,   // Cờ dữ liệu hợp lệ
    output reg [7:0] sensor_data // Dữ liệu cảm biến (8-bit)
);
    // Giả sử cấu trúc frame:
    // Byte0: frame[23:16] = Header (8'hAA)
    // Byte1: frame[15:8]  = Sensor Data
    // Byte2: frame[7:0]   = Checksum (định nghĩa: checksum = Sensor Data)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_data  <= 1'b0;
            sensor_data <= 8'd0;
        end else if (valid_frame) begin
            if (frame[23:16] == 8'hAA && frame[7:0] == frame[15:8]) begin
                sensor_data <= frame[15:8];
                valid_data  <= 1'b1;
            end else begin
                valid_data  <= 1'b0;
            end
        end else begin
            valid_data <= 1'b0;
        end
    end

endmodule
