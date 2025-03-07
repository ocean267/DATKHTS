// Module chuyển đổi sensor_data (8-bit) thành chuỗi ASCII dạng "SENS: XXX       "
// Tổng chuỗi gồm 16 ký tự (16x8 = 128-bit).
module sensor_data_to_ascii(
    input  [7:0] sensor_data,     // Dữ liệu cảm biến (ví dụ: giá trị khoảng cách 0-255)
    output [127:0] ascii_row      // Chuỗi ASCII kết quả (16 ký tự)
);

  // Các biến trung gian dùng để tách hàng trăm, hàng chục, hàng đơn vị
  reg [7:0] hundreds;
  reg [7:0] tens;
  reg [7:0] ones;
  reg [7:0] remainder;
  
  // Khối always kết hợp: tính các chữ số từ sensor_data
  always @(*) begin
      // Tính chữ số hàng trăm: vì sensor_data max = 255 nên chỉ có thể là 0, 1 hoặc 2
      hundreds  = sensor_data / 100;
      remainder = sensor_data - (sensor_data / 100) * 100;
      tens      = remainder / 10;
      ones      = remainder - (remainder / 10) * 10;
  end
  
  // Ghép chuỗi kết quả:
  // "SENS: " là tiền tố (6 ký tự),
  // tiếp theo là 3 chữ số (chuyển từ số sang mã ASCII bằng cách cộng 8'd48),
  // và cuối cùng là phần đệm (7 khoảng trắng) để đủ 16 ký tự.
  assign ascii_row = { "SENS: ", 
                       8'd48 + hundreds, 
                       8'd48 + tens, 
                       8'd48 + ones, 
                       "       " };

endmodule
