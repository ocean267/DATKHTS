module lcd_display(
    input           clk_1MHz,               // 1 MHz = 1us clock    
    input           rst_n,                  // active low reset
    input           ena,                    // enable write (nếu luôn update thì có thể để 1'b1)
    input           done_write,             // cờ báo write hoàn thành từ module ghi I2C
    input  [127:0]  row1,                   // dữ liệu hàng 1 (cập nhật từ cảm biến)
    input  [127:0]  row2,                   // dữ liệu hàng 2 (cập nhật từ cảm biến)
    output reg [7:0] data,                  // dữ liệu gửi đến module ghi I2C
    output          cmd_data,               // 0 = lệnh, 1 = dữ liệu hiển thị
    output reg      ena_write               // cờ cho phép ghi dữ liệu (I2C)
);

    localparam DELAY = 50;    // độ trễ 50us giữa các byte

    // Định nghĩa các trạng thái FSM
    localparam  WaitEn    = 0,
                Write     = 1,
                WaitWrite = 3,
                WaitDelay = 4,
                Done      = 5;

    reg [2:0] state, next_state;
    reg [20:0] cnt;       // bộ đếm microsecond
    reg        cnt_clr;   // cờ xóa bộ đếm

    // Mảng lưu các byte lệnh và dữ liệu cho LCD (40 byte)
    wire [7:0] cmd_data_array [0:39];

    assign cmd_data_array[0]  = 8'h02;    // thiết lập 4-bit mode
    assign cmd_data_array[1]  = 8'h28;    // khởi tạo LCD 16x2 ở chế độ 4-bit
    assign cmd_data_array[2]  = 8'h0C;    // bật hiển thị, tắt con trỏ
    assign cmd_data_array[3]  = 8'h06;    // tự động tăng con trỏ
    assign cmd_data_array[4]  = 8'h01;    // xóa màn hình
    assign cmd_data_array[5]  = 8'h80;    // đặt con trỏ ở hàng 1, vị trí đầu
    assign cmd_data_array[22] = 8'hC0;    // đặt con trỏ ở hàng 2, vị trí đầu

    // Gán dữ liệu hiển thị từ row1, row2 vào mảng
    generate
        genvar i;
        for (i = 1; i < 17; i = i + 1) begin: for_name
            assign cmd_data_array[22 - i] = row1[(i*8)-1 -: 8]; // hàng 1
            assign cmd_data_array[39 - i] = row2[(i*8)-1 -: 8]; // hàng 2
        end
    endgenerate

    reg [5:0] ptr;  // con trỏ chỉ vào cmd_data_array

    // Xác định tín hiệu cmd_data: nếu ptr ≤ 6 hoặc ptr = 23 thì gửi lệnh (0), ngược lại là dữ liệu (1)
    assign cmd_data = (ptr <= 6'd6 || ptr == 6'd23) ? 1'b0 : 1'b1;

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
            state <= WaitEn;
        else
            state <= next_state;
    end

    // Logic chuyển trạng thái
    always @(*) begin
        if (!rst_n)
            next_state = WaitEn;
        else begin
            case (state)
                WaitEn: begin
                    // Nếu ena kích hoạt (hoặc nếu bạn luôn update, ena có thể đặt là 1)
                    next_state = ena ? Write : WaitEn;
                end
                Write: begin
                    next_state = WaitWrite;
                end
                WaitWrite: begin
                    // Chờ cho module ghi I2C báo done
                    next_state = done_write ? WaitDelay : WaitWrite;
                end
                WaitDelay: begin
                    // Sau một khoảng trễ, nếu chưa ghi hết tất cả 40 byte thì chuyển sang Write để gửi byte tiếp theo,
                    // còn nếu đã ghi byte cuối (ptr == 39) thì chuyển sang Done
                    next_state = (cnt == DELAY) ? ((ptr == 6'd39) ? Done : Write) : WaitDelay;
                end
                Done: begin
                    // Ở trạng thái Done, sau khoảng trễ thì quay trở lại WaitEn để bắt đầu cập nhật lại LCD
                    next_state = (cnt == DELAY) ? WaitEn : Done;
                end
                default: next_state = WaitEn;
            endcase
        end
    end

    // Logic xuất tín hiệu dựa trên trạng thái hiện tại
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n) begin
            cnt_clr   <= 1'b1;
            ena_write <= 1'b0;
            data      <= 8'd0;
        end else begin
            case (state)
                WaitEn: begin
                    cnt_clr   <= 1'b1; // xóa bộ đếm
                    ena_write <= 1'b0; // không kích hoạt ghi
                end
                Write: begin
                    cnt_clr   <= 1'b1; // xóa bộ đếm
                    data      <= cmd_data_array[ptr]; // nạp byte dữ liệu/ lệnh cần ghi
                    ena_write <= 1'b1; // kích hoạt ghi
                end
                WaitWrite: begin
                    ena_write <= 1'b0; // tắt tín hiệu ghi trong lúc chờ báo done
                end
                WaitDelay: begin
                    cnt_clr   <= 1'b0; // tiếp tục đếm trễ
                    ena_write <= 1'b0;
                end
                Done: begin
                    cnt_clr   <= 1'b1; // xóa bộ đếm
                    ena_write <= 1'b0;
                end
                default: begin
                    cnt_clr   <= 1'b0;
                    ena_write <= 1'b0;
                end
            endcase
        end
    end

    // Cập nhật con trỏ ptr: Khi ở trạng thái Write thì tăng ptr, và khi ở trạng thái Done thì đặt ptr về 0 để làm mới dữ liệu
    always @(posedge clk_1MHz or negedge rst_n) begin
        if (!rst_n)
            ptr <= 6'd0;
        else if (state == Write)
            ptr <= ptr + 1'b1;
        else if (state == Done)
            ptr <= 6'd0;
    end

endmodule
