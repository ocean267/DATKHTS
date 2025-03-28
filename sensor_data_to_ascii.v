// Module chuyển đổi sensor_data (16-bit) thành chuỗi ASCII dạng "SENS: XXXX      "
// Tổng chuỗi gồm 16 ký tự (16 x 8 = 128-bit).
module sensor_data_to_ascii(
    input  [15:0] sensor_data,   // Dữ liệu cảm biến 16-bit (0 - 65535)
    output [127:0] ascii_row     // Chuỗi ASCII kết quả (16 ký tự)
);

  // Ta chọn hiển thị 4 chữ số bằng cách chia sensor_data cho 10
  // (để giá trị hiển thị nằm trong khoảng 0 - 9999).
  reg [15:0] value_to_display;
  reg [7:0] thousands; // Chữ số hàng nghìn
  reg [7:0] hundreds;  // Chữ số hàng trăm
  reg [7:0] tens;      // Chữ số hàng chục
  reg [7:0] ones;      // Chữ số hàng đơn vị
  reg [15:0] r1, r2;

  always @(*) begin
      // Chọn giá trị hiển thị: chia cho 10 (có thể điều chỉnh nếu cần)
      value_to_display = sensor_data / 10;

      // Tách các chữ số
      thousands = value_to_display / 1000;
      r1        = value_to_display % 1000;
      hundreds  = r1 / 100;
      r2        = r1 % 100;
      tens      = r2 / 10;
      ones      = r2 % 10;
  end

  // Ghép chuỗi kết quả:
  // "SENS: " là tiền tố (6 ký tự),
  // sau đó là 4 chữ số (từng chữ số chuyển sang ASCII bằng cách cộng 8'd48),
  // và cuối cùng là 6 khoảng trắng để đủ 16 ký tự.
  assign ascii_row = { "SENS: ",
                       8'd48 + thousands,
                       8'd48 + hundreds,
                       8'd48 + tens,
                       8'd48 + ones,
                       "      " };

endmodule
