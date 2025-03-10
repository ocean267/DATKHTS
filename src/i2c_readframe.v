module i2c_readframe(
    input       clk_1MHz,               // Clock 1 MHz (1μs/chu kỳ)
    input       rst_n,                  // Reset active low
    input       en_read,                // Cho phép đọc
    input       start_frame,            // Cờ bắt đầu frame
    input       stop_frame,             // Cờ dừng frame
    inout       sda,                    // Đường dữ liệu I2C (bidirectional)
    output reg  scl,                    // Đường clock I2C
    output reg [23:0] frame_data,        // Frame 3 byte (24-bit): {Header, Sensor Data, Checksum}
    output      done,                   // Cờ báo hoàn thành đọc
    output reg  sda_en                  // Điều khiển hướng của SDA: 1 = master drive, 0 = giải phóng
);

    localparam DELAY = 10;  // Độ trễ 10μs cho mỗi trạng thái
    reg [7:0] cnt;          // Sử dụng 8-bit cho bộ đếm DELAY nhỏ
    reg       cnt_clr;

    // FSM States (đã loại bỏ trạng thái READ_DONE không dùng)
    localparam WAIT_EN    = 0,
               PRE_START  = 1,
               START      = 2,
               AFTER_START= 3,
               PRE_READ   = 4,
               READ_LOW   = 5,
               READ_HIGH  = 6,
               NEXT_BYTE  = 7,
               DONE       = 8;
               
    reg [3:0] state, next_state;
    reg [3:0] bit_cnt;      // Bộ đếm bit trong một byte (0 đến 7)
    reg [1:0] byte_cnt;     // Bộ đếm byte (0 đến 2)
    reg       sda_out;
    wire      sda_in;
    
    // Tri-state cho SDA:
    // Khi sda_en = 1, nếu sda_out = 0 thì SDA kéo xuống; nếu sda_out = 1 thì SDA ở high impedance.
    // Khi sda_en = 0, SDA luôn high impedance để đọc từ bus.
    assign sda = sda_en ? ((sda_out == 1'b0) ? 1'b0 : 1'bz) : 1'bz;
    assign sda_in = sda;
    
    // Bộ đếm microsecond
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n)
            cnt <= 8'd0;
        else if (cnt_clr)
            cnt <= 8'd0;
        else
            cnt <= cnt + 1'b1;
    end
    
    // Cập nhật trạng thái FSM
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n)
            state <= WAIT_EN;
        else
            state <= next_state;
    end
    
    // Logic chuyển trạng thái
    always @(*) begin
        case (state)
            WAIT_EN:    next_state = (en_read && start_frame) ? PRE_START : WAIT_EN;
            PRE_START:  next_state = (cnt == DELAY-1) ? START : PRE_START;
            START:      next_state = (cnt == DELAY-1) ? AFTER_START : START;
            AFTER_START:next_state = (cnt == DELAY-1) ? PRE_READ : AFTER_START;
            PRE_READ:   next_state = (cnt == DELAY-1) ? READ_LOW : PRE_READ;
            READ_LOW:   next_state = (cnt == DELAY-1) ? READ_HIGH : READ_LOW;
            READ_HIGH:  next_state = (cnt == DELAY-1) ? 
                                       ((bit_cnt == 4'd7) ? NEXT_BYTE : READ_LOW) : READ_HIGH;
            NEXT_BYTE:  next_state = (cnt == DELAY-1) ? 
                                       ((byte_cnt == 2'd2) ? DONE : PRE_READ) : NEXT_BYTE;
            DONE:       next_state = (cnt == DELAY-1) ? WAIT_EN : DONE;
            default:    next_state = WAIT_EN;
        endcase
    end
    
    // Logic xuất tín hiệu và thu thập dữ liệu
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n) begin
            sda_en     <= 1'b1;
            sda_out    <= 1'b1;
            scl        <= 1'b1;
            cnt_clr    <= 1'b1;
            bit_cnt    <= 4'd0;
            byte_cnt   <= 2'd0;
            frame_data <= 24'd0;
        end else begin
            cnt_clr <= 1'b0; // Mặc định không xóa bộ đếm
            case (state)
                WAIT_EN: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    scl     <= 1'b1;
                    // Reset counters khi ở trạng thái chờ
                    bit_cnt  <= 4'd0;
                    byte_cnt <= 2'd0;
                    if (cnt == DELAY-1)
                        cnt_clr <= 1'b1;
                end
                PRE_START: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    scl     <= 1'b1;
                    if (cnt == DELAY-1)
                        cnt_clr <= 1'b1;
                end
                START: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b0;   // Kéo SDA xuống để tạo start condition
                    scl     <= 1'b1;
                    if (cnt == DELAY-1)
                        cnt_clr <= 1'b1;
                end
                AFTER_START: begin
                    sda_en  <= 1'b1;
                    scl     <= 1'b0;   // Sau start, kéo SCL xuống
                    if (cnt == DELAY-1)
                        cnt_clr <= 1'b1;
                end
                PRE_READ: begin
                    sda_en  <= 1'b0;   // Giải phóng SDA để đọc dữ liệu
                    scl     <= 1'b0;
                    if (cnt == DELAY-1)
                        cnt_clr <= 1'b1;
                end
                READ_LOW: begin
                    sda_en  <= 1'b0;
                    scl     <= 1'b0;
                    if (cnt == DELAY-1)
                        cnt_clr <= 1'b1;
                end
                READ_HIGH: begin
                    sda_en  <= 1'b0;
                    scl     <= 1'b1;   // Khi SCL lên, dữ liệu SDA đã ổn định
                    if (cnt == DELAY-1) begin
                        // Lấy mẫu bit từ sda_in và lưu vào vị trí tương ứng trong frame_data
                        frame_data[((2 - byte_cnt)*8) + (7 - bit_cnt)] <= sda_in;
                        // Tăng bộ đếm bit
                        bit_cnt <= bit_cnt + 1;
                        cnt_clr <= 1'b1;
                    end
                end
                NEXT_BYTE: begin
                    // Sau khi đọc xong 1 byte, reset bit counter và tăng byte counter
                    bit_cnt  <= 4'd0;
                    byte_cnt <= byte_cnt + 1;
                    cnt_clr  <= 1'b1;
                end
                DONE: begin
                    scl    <= 1'b1;
                    sda_en <= 1'b1;
                    cnt_clr <= 1'b1;
                end
                default: ;
            endcase

            // Ở trạng thái WAIT_EN hoặc DONE, reset bit counter
            if (state == WAIT_EN || state == DONE)
                bit_cnt <= 4'd0;
        end
    end

    assign done = (state == DONE);
    
endmodule
