module top(
    input           clk,          // Clock hệ thống (ví dụ: 100 MHz)
    input           rst_n,        // active low reset

    // I2C bus dành cho LCD
    output          lcd_scl,      
    inout           lcd_sda,      

    // I2C bus dành cho cảm biến
    output          sensor_scl,   
    inout           sensor_sda    
);

    // -------------------------------
    // Sinh xung 1MHz từ clock hệ thống qua module clk_divider
    // -------------------------------
    wire clk_1MHz;
    clk_divider clk_div_gen(
        .clk        (clk),
        .rst_n      (rst_n),
        .clk_1MHz   (clk_1MHz)
    );

    // -------------------------------
    // Phần giao tiếp cảm biến:
    // Module i2c_readframe đọc dữ liệu từ cảm biến qua giao thức I2C
    // -------------------------------
    wire sensor_req;
    wire sensor_done;
    // Khai báo wire cho frame 24-bit (nếu module i2c_readframe đã được cập nhật)
    wire [23:0] sensor_frame;

    // Ở đây sensor_req luôn kích hoạt đọc, nhưng trong ứng dụng thực tế bạn có thể dùng FSM điều khiển.
    assign sensor_req = 1'b1;

    // Tín hiệu start_frame và stop_frame (demo đơn giản gán sensor_req)
    wire sensor_start_frame;
    wire sensor_stop_frame;
    assign sensor_start_frame = sensor_req;
    assign sensor_stop_frame  = sensor_req;

    // Dùng wire dummy cho cổng sda_en nếu không sử dụng
    wire dummy_sda_en;
    
    i2c_readframe i2c_readframe_inst(
        .clk_1MHz    (clk_1MHz),
        .rst_n       (rst_n),
        .en_read     (sensor_req),
        .start_frame (sensor_start_frame),
        .stop_frame  (sensor_stop_frame),
        .sda         (sensor_sda),
        .scl         (sensor_scl),
        .frame_data  (sensor_frame), // Thay đổi từ "data" sang "frame_data"
        .done        (sensor_done),
        .sda_en      (dummy_sda_en)
    );

    // -------------------------------
    // Phân tích frame để lấy sensor_data:
    // Module frame_parser nhận frame 24-bit và kiểm tra hợp lệ, xuất sensor_data (8-bit)
    // -------------------------------
    wire valid_data;
    wire [7:0] sensor_data;
    frame_parser frame_parser_inst (
        .clk         (clk_1MHz),
        .rst_n       (rst_n),
        .valid_frame (sensor_done),
        .frame       (sensor_frame),
        .valid_data  (valid_data),
        .sensor_data (sensor_data)
    );

    // -------------------------------
    // Chuyển đổi sensor_data thành chuỗi ASCII bằng module sensor_data_to_ascii
    // -------------------------------
    wire [127:0] sensor_ascii;
    sensor_data_to_ascii sensor_to_ascii_inst (
        .sensor_data(sensor_data),
        .ascii_row  (sensor_ascii)
    );

    // -------------------------------
    // Tạo nội dung hiển thị cho 2 dòng của LCD:
    // - Row1: hiển thị trạng thái (ví dụ: "SAFE" hoặc "DANGER") dựa trên giá trị sensor_data.
    // - Row2: hiển thị giá trị cảm biến dưới dạng ASCII từ module sensor_data_to_ascii.
    // -------------------------------
    wire [127:0] row1;
    wire [127:0] row2;
    assign row1 = (sensor_data > 8'd100) ?  "SAFE            " : "DANGER          " ;
    assign row2 = sensor_ascii;

    // -------------------------------
    // Phần giao tiếp LCD:
    // Module lcd_display tạo ra dữ liệu hiển thị và module lcd_write_cmd_data chuyển giao tiếp I2C đến LCD.
    // -------------------------------
    wire        lcd_done;
    wire [7:0]  lcd_data;
    wire        lcd_cmd_data;
    wire        lcd_ena;    // Yêu cầu giao tiếp I2C từ LCD

    lcd_display lcd_display_inst(
        .clk_1MHz   (clk_1MHz),
        .rst_n      (rst_n),
        .ena        (1'b1),         // Luôn làm mới LCD
        .done_write (lcd_done),
        .row1       (row1),
        .row2       (row2),
        .data       (lcd_data),
        .cmd_data   (lcd_cmd_data),
        .ena_write  (lcd_ena)
    );

    lcd_write_cmd_data lcd_write_cmd_data_inst(
        .clk_1MHz   (clk_1MHz),
        .rst_n      (rst_n),
        .data       (lcd_data),
        .cmd_data   (lcd_cmd_data),
        .ena        (lcd_ena),
        .i2c_addr   (7'h27),
        .sda        (lcd_sda),
        .scl        (lcd_scl),
        .done       (lcd_done)
    );

endmodule
