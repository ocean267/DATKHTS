module i2c_readframe(
    input       clk_1MHz,               // Clock 1 MHz, mỗi chu kỳ 1μs
    input       rst_n,                  // Reset active low
    input       en_read,                // Cho phép đọc
    input       start_frame,            // Cờ bắt đầu frame
    input       stop_frame,             // Cờ dừng frame
    inout       sda,                    // Đường dữ liệu I2C (bidirectional)
    output reg  scl,                    // Đường clock I2C
    output reg [7:0] data,               // Dữ liệu đọc được từ slave
    output      done,                   // Cờ báo hoàn thành đọc
    output reg  sda_en                  // Điều khiển hướng của SDA: 1 là master điều khiển, 0 là giải phóng để đọc
);

    localparam  DELAY       = 10;       // Độ trễ 10μs cho mỗi trạng thái
    reg [20:0]  cnt;                    // Bộ đếm microsecond
    reg         cnt_clr;                // Cờ xóa bộ đếm

    // Định nghĩa các trạng thái FSM
    localparam  WAIT_EN    = 0,
                PRE_START  = 1,
                START      = 2,
                AFTER_START= 3,
                PRE_READ   = 4,
                READ_LOW   = 5,
                READ_HIGH  = 6,
                READ_DONE  = 7,
                PRE_NACK   = 8,
                NACK1      = 9,
                NACK2      = 10,
                PRE_STOP   = 11,
                STOP       = 12,
                DONE       = 13;
                
    reg [3:0] state, next_state;
    reg [3:0] bit_cnt;                // Bộ đếm bit (từ 0 đến 8)
    reg       sda_out;                // Giá trị điều khiển ra cho SDA (chỉ có tác dụng khi sda_en = 1)
    wire      sda_in;                 // Giá trị đọc từ SDA

    // Tri-state cho đường SDA:
    // Khi sda_en = 1, nếu sda_out = 0 thì SDA được kéo xuống; nếu sda_out = 1 thì SDA ở high impedance (để pull-up kéo lên).
    // Khi sda_en = 0, SDA luôn ở high impedance để cho phép đọc từ bus.
    assign sda = sda_en ? ((sda_out == 1'b0) ? 1'b0 : 1'bz) : 1'bz;
    assign sda_in = sda;

    // Bộ đếm microsecond
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n)
            cnt <= 21'd0;
        else if (cnt_clr)
            cnt <= 21'd0;
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
        if (!rst_n)
            next_state = WAIT_EN;
        else begin
            case (state)
                WAIT_EN:    next_state = (en_read && start_frame) ? PRE_START : WAIT_EN;
                PRE_START:  next_state = (cnt == DELAY) ? START : PRE_START;
                START:      next_state = (cnt == DELAY) ? AFTER_START : START;
                AFTER_START:next_state = (cnt == DELAY) ? PRE_READ : AFTER_START;
                PRE_READ:   next_state = (cnt == DELAY) ? READ_LOW : PRE_READ;
                READ_LOW:   next_state = (cnt == DELAY) ? READ_HIGH : READ_LOW;
                READ_HIGH:  next_state = (cnt == DELAY) ? ((bit_cnt == 4'd8) ? READ_DONE : READ_LOW) : READ_HIGH;
                READ_DONE:  next_state = (cnt == DELAY) ? PRE_NACK : READ_DONE;
                PRE_NACK:   next_state = (cnt == DELAY) ? NACK1 : PRE_NACK;
                NACK1:      next_state = (cnt == DELAY) ? NACK2 : NACK1;
                NACK2:      next_state = (cnt == DELAY) ? PRE_STOP : NACK2;
                PRE_STOP:   next_state = (cnt == DELAY) ? STOP : PRE_STOP;
                STOP:       next_state = (cnt == DELAY) ? DONE : STOP;
                DONE:       next_state = (cnt == DELAY) ? WAIT_EN : DONE;
                default:    next_state = WAIT_EN;
            endcase
        end
    end

    // Logic xuất và điều khiển các tín hiệu dựa trên trạng thái hiện tại
    // (Loại bỏ các gán cho data và bit_cnt để tránh xung đột với always block thu thập dữ liệu bên dưới)
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n) begin
            sda_en  <= 1'b1;
            sda_out <= 1'b1;
            scl     <= 1'b1;
            cnt_clr <= 1'b1;
        end else begin
            case (state)
                WAIT_EN: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                PRE_START: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                START: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b0;      // Tạo điều kiện start: kéo SDA xuống khi SCL cao
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                AFTER_START: begin
                    sda_en  <= 1'b1;
                    scl     <= 1'b0;      // Sau start, kéo SCL xuống để chuẩn bị truyền
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                PRE_READ: begin
                    sda_en  <= 1'b0;      // Giải phóng SDA để chuyển sang chế độ đọc
                    scl     <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                READ_LOW: begin
                    sda_en  <= 1'b0;      // Đảm bảo SDA ở chế độ input
                    scl     <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                READ_HIGH: begin
                    sda_en  <= 1'b0;
                    scl     <= 1'b1;      // Khi SCL lên, lấy mẫu giá trị của SDA (bit dữ liệu)
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                READ_DONE: begin
                    // Sau khi nhận đủ 8 bit, chuẩn bị gửi NACK
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;      // NACK: giữ SDA ở mức cao
                    scl     <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                PRE_NACK: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    scl     <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                NACK1: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    scl     <= 1'b1;      // Tạo xung clock cho NACK
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                NACK2: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    scl     <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                PRE_STOP: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b0;      // Chuẩn bị dừng: giữ SDA thấp khi SCL cao
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                STOP: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;      // Điều kiện stop: đưa SDA lên khi SCL ở mức cao
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                DONE: begin
                    cnt_clr <= 1'b1;
                end
                default: begin
                    cnt_clr <= 1'b0;
                end
            endcase
        end
    end

    // Bộ đếm bit và thu thập dữ liệu:
    // Khi ở trạng thái READ_HIGH, tại đầu chu kỳ (cnt==0) ta lấy mẫu giá trị của SDA và lưu vào vị trí tương ứng trong data.
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt <= 4'd0;
            data    <= 8'd0;
        end else begin
            if (state == READ_HIGH && cnt == 0) begin
                data[7 - bit_cnt] <= sda_in;   // Lấy mẫu bit dữ liệu
                bit_cnt <= bit_cnt + 1;
            end else if (state == WAIT_EN || state == DONE) begin
                bit_cnt <= 4'd0;
            end
        end
    end

    // Cờ báo hoàn thành đọc
    assign done = (state == DONE);

endmodule
