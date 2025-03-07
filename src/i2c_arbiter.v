module i2c_arbiter(
    input        clk,
    input        rst_n,
    // Yêu cầu từ các master:
    input        req_lcd,      // yêu cầu từ module LCD (ví dụ: tín hiệu ena từ lcd_display)
    input        req_sensor,   // yêu cầu từ module đọc cảm biến
    // Tín hiệu báo giao dịch hoàn thành từ các master:
    input        lcd_done,
    input        sensor_done,
    // Tín hiệu cấp quyền truy cập bus:
    output reg   grant_lcd,
    output reg   grant_sensor
);

    // FSM với ba trạng thái: IDLE, LCD_ACTIVE, SENSOR_ACTIVE.
    localparam IDLE         = 2'd0;
    localparam LCD_ACTIVE   = 2'd1;
    localparam SENSOR_ACTIVE= 2'd2;
    
    reg [1:0] state, next_state;
    // Flip-flop cho cơ chế round-robin: 0 nếu lần trước phục vụ LCD, 1 nếu phục vụ cảm biến.
    reg       last_grant;  

    // Cập nhật trạng thái và flip-flop round-robin
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            last_grant <= 1'b0;
        end else begin
            state <= next_state;
            if(state == LCD_ACTIVE && lcd_done)
                last_grant <= 1'b0;  // LCD vừa được phục vụ
            else if(state == SENSOR_ACTIVE && sensor_done)
                last_grant <= 1'b1;  // Cảm biến vừa được phục vụ
        end
    end

    // FSM chuyển trạng thái và xuất grant
    always @(*) begin
        // Mặc định không cấp quyền
        grant_lcd   = 1'b0;
        grant_sensor= 1'b0;
        next_state  = state;
        case (state)
            IDLE: begin
                // Nếu chỉ có một master yêu cầu, cấp quyền trực tiếp.
                if (req_lcd && !req_sensor) begin
                    next_state = LCD_ACTIVE;
                    grant_lcd  = 1'b1;
                end else if (req_sensor && !req_lcd) begin
                    next_state = SENSOR_ACTIVE;
                    grant_sensor = 1'b1;
                end else if (req_lcd && req_sensor) begin
                    // Nếu cả hai cùng yêu cầu, dùng cơ chế round-robin.
                    if (last_grant == 1'b0) begin
                        next_state = SENSOR_ACTIVE;
                        grant_sensor = 1'b1;
                    end else begin
                        next_state = LCD_ACTIVE;
                        grant_lcd = 1'b1;
                    end
                end else begin
                    next_state = IDLE;
                end
            end
            LCD_ACTIVE: begin
                grant_lcd = 1'b1;
                if(lcd_done)
                    next_state = IDLE;
            end
            SENSOR_ACTIVE: begin
                grant_sensor = 1'b1;
                if(sensor_done)
                    next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

endmodule
