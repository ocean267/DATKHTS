module top(
    input           clk,          // Clock hệ thống (ví dụ: 100 MHz)
    input           rst_n,        // Active low reset

    // I2C bus dành cho LCD
    output          lcd_scl,      
    inout           lcd_sda,      

    // I2C bus dành cho cảm biến (VL53L0X chỉ có 4 chân: VIN, GND, SDA, SCL)
    // Module vl53l0x_init tự quản lý giao tiếp I2C với cảm biến.
    output          sensor_scl,   
    inout           sensor_sda    
);

    // ------------------------------------------
    // Sinh xung 1MHz từ clock hệ thống qua clk_divider
    // ------------------------------------------
    wire clk_1MHz;
    clk_divider clk_div_gen (
        .clk      (clk),
        .rst_n    (rst_n),
        .clk_1MHz (clk_1MHz)
    );

    // ------------------------------------------
    // Tín hiệu tự động kích hoạt khởi tạo và đọc dữ liệu từ sensor
    // ------------------------------------------
    wire auto_start_init;
    wire auto_start_read;

    // Module điều khiển tự động: tự động phát ra tín hiệu start_init và start_read theo trình tự FSM
    sensor_auto_ctrl auto_ctrl_inst (
        .clk        (clk_1MHz),
        .rst_n      (rst_n),
        .init_done  (init_done),
        .read_done  (read_done),
        .start_init (auto_start_init),
        .start_read (auto_start_read)
    );

    // ------------------------------------------
    // Module vl53l0x_init: Khởi tạo & đọc dữ liệu từ VL53L0X
    // Module này tự thực hiện các giao dịch I2C cần thiết để:
    // - Kiểm tra Device ID
    // - Cấu hình, nạp tuning settings, cấu hình ngắt, hiệu chuẩn
    // - Và cuối cùng, kích hoạt phép đo & đọc kết quả
    // Dữ liệu đo được (16-bit) được lưu vào cổng "range".
    wire         sensor_init_i2c_start;
    wire         sensor_init_i2c_rw;
    wire [7:0]   sensor_init_i2c_reg_addr;
    wire [7:0]   sensor_init_i2c_data_out;
    wire         init_done;
    wire         read_done;
    wire         error;
    wire [15:0]  range;
    
    vl53l0x_init sensor_inst (
        .clk           (clk_1MHz),
        .rst_n         (rst_n),
        .start_init    (auto_start_init), // sử dụng tín hiệu tự động từ sensor_auto_ctrl
        .start_read    (auto_start_read), // sử dụng tín hiệu tự động từ sensor_auto_ctrl
        .init_done     (init_done),
        .read_done     (read_done),
        .error         (error),
        .range         (range),
        .i2c_start     (sensor_init_i2c_start),
        .i2c_rw        (sensor_init_i2c_rw),
        .i2c_reg_addr  (sensor_init_i2c_reg_addr),
        .i2c_data_out  (sensor_init_i2c_data_out),
        .i2c_done      (),         // Giả sử module này tự quản lý I2C nội bộ
        .i2c_data_in   ()          // Giả sử module này tự quản lý I2C nội bộ
    );
    
    // ------------------------------------------
    // Tang Nano 9K lấy dữ liệu đo trực tiếp từ cổng "range" của vl53l0x_init
    // ------------------------------------------
    wire [15:0] sensor_data;
    assign sensor_data = range ;
    
    // ------------------------------------------
    // Chuyển dữ liệu đo thành chuỗi ASCII để hiển thị lên LCD
    // ------------------------------------------
    wire [127:0] ascii_data;
    sensor_data_to_ascii sensor_to_ascii_inst (
        .sensor_data(sensor_data),
        .ascii_row(ascii_data)
    );
 
    // Hai dòng hiển thị LCD: dòng 1 hiển thị thông điệp, dòng 2 hiển thị kết quả đo (ASCII)
    wire [127:0] row1;
    wire [127:0] row2;
    assign row1 = (sensor_data > 16'd100) ? "SAFE            " : "DANGER          ";
    assign row2 = ascii_data;
    
    // ------------------------------------------
    // Giao tiếp LCD: Hiển thị dữ liệu lên màn hình qua I2C
    // ------------------------------------------
    wire        lcd_done;
    wire [7:0]  lcd_data;
    wire        lcd_cmd_data;
    wire        lcd_ena;
    
    lcd_display lcd_display_inst (
        .clk_1MHz(clk_1MHz),
        .rst_n   (rst_n),
        .ena     (1'b1),         // Luôn làm mới LCD
        .done_write(lcd_done),
        .row1    (row1),
        .row2    (row2),
        .data    (lcd_data),
        .cmd_data(lcd_cmd_data),
        .ena_write(lcd_ena)
    );
    
    lcd_write_cmd_data lcd_write_cmd_data_inst (
        .clk_1MHz(clk_1MHz),
        .rst_n   (rst_n),
        .data    (lcd_data),
        .cmd_data(lcd_cmd_data),
        .ena     (lcd_ena),
        .i2c_addr(7'h27),
        .sda     (lcd_sda),
        .scl     (lcd_scl),
        .done    (lcd_done)
    );
    
    // ------------------------------------------
    // Nếu cần, bạn có thể kết nối tín hiệu I2C của cảm biến từ module vl53l0x_init
    // với bus cảm biến, ví dụ:
    // assign sensor_scl = <port từ vl53l0x_init, nếu có>;
    // assign sensor_sda = <port từ vl53l0x_init, nếu có>;
    // Ở đây, ta giả sử module vl53l0x_init tự quản lý giao tiếp I2C với cảm biến.
    // ------------------------------------------

endmodule
