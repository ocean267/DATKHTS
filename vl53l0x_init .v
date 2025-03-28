// Module vl53l0x_controller: thực hiện khởi tạo và đọc khoảng cách từ sensor VL53L0X
// Các chức năng bao gồm: kiểm tra Device ID, data_init, load_default_tuning_settings,
// configure_interrupt, static_init, hiệu chuẩn (calibration) và đọc khoảng cách.

module vl53l0x_init (
    input         clk,
    input         rst_n,
    // Các tín hiệu điều khiển bên ngoài
    input         start_init,   // kích hoạt khởi tạo sensor
    input         start_read,   // kích hoạt đọc khoảng cách (sau khi init_done)
    output reg    init_done,    // báo hiệu khởi tạo hoàn tất
    output reg    read_done,    // báo hiệu đọc khoảng cách hoàn tất
    output reg    error,        // báo hiệu lỗi trong quá trình giao dịch
    output reg [15:0] range,    // dữ liệu khoảng cách đọc được

    // Giao diện I2C (điều khiển giao dịch I2C)
    output reg        i2c_start,
    output reg        i2c_rw,       // 0: write, 1: read
    output reg [7:0]  i2c_reg_addr,
    output reg [7:0]  i2c_data_out,
    input             i2c_done,
    input      [7:0]  i2c_data_in
);

  //-------------------------------------------------------------------------
  // ĐỊNH NGHĨA CÁC ĐỊA CHỈ THÀNH GHI và HẰNG
  //-------------------------------------------------------------------------
  localparam REG_IDENTIFICATION_MODEL_ID         = 8'hC0;
  localparam REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV  = 8'h89;
  localparam REG_MSRC_CONFIG_CONTROL              = 8'h60;
  localparam REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 8'h44;
  localparam REG_SYSTEM_SEQUENCE_CONFIG           = 8'h01;
  localparam REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = 8'h4F;
  localparam REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 8'h4E;
  localparam REG_GLOBAL_CONFIG_REF_EN_START_SELECT = 8'hB6;
  localparam REG_SYSTEM_INTERRUPT_CONFIG_GPIO     = 8'h0A;
  localparam REG_GPIO_HV_MUX_ACTIVE_HIGH          = 8'h84;
  localparam REG_SYSTEM_INTERRUPT_CLEAR           = 8'h0B;
  localparam REG_RESULT_INTERRUPT_STATUS          = 8'h13;
  localparam REG_SYSRANGE_START                   = 8'h00;
  localparam REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0  = 8'hB0;
  localparam REG_RESULT_RANGE_STATUS              = 8'h14;

  localparam VL53L0X_EXPECTED_DEVICE_ID = 8'hEE;
  localparam VL53L0X_OUT_OF_RANGE       = 16'hFFFF; // định nghĩa giá trị out-of-range

  // Các bước đo sequence (các giá trị bit)
  localparam RANGE_SEQUENCE_STEP_DSS       = 8'h28;
  localparam RANGE_SEQUENCE_STEP_PRE_RANGE = 8'h40;
  localparam RANGE_SEQUENCE_STEP_FINAL_RANGE = 8'h80;
  // Tổ hợp sequence cho hoạt động đo
  localparam SEQUENCE_ENABLE = RANGE_SEQUENCE_STEP_DSS + RANGE_SEQUENCE_STEP_PRE_RANGE + RANGE_SEQUENCE_STEP_FINAL_RANGE; // = 0xE8

  //-------------------------------------------------------------------------
  // FSM State Encoding
  //-------------------------------------------------------------------------
  localparam S_IDLE                        = 8'd0;
  // --- INIT: kiểm tra boot
  localparam S_INIT_READ_DEVICE_ID         = 8'd1;
  localparam S_INIT_WAIT_DEVICE_ID         = 8'd2;
  localparam S_INIT_CHECK_DEVICE_ID        = 8'd3;
  // --- DATA_INIT: thực hiện các giao dịch ban đầu
  localparam S_DATA_INIT_READ_0            = 8'd4;
  localparam S_DATA_INIT_WAIT_READ_0       = 8'd5;
  localparam S_DATA_INIT_WRITE_0           = 8'd6;
  localparam S_DATA_INIT_WAIT_WRITE_0      = 8'd7;
  localparam S_DATA_INIT_WRITE_1           = 8'd8;   // write to reg 0x88, value 0x00
  localparam S_DATA_INIT_WAIT_WRITE_1      = 8'd9;
  localparam S_DATA_INIT_WRITE_2           = 8'd10;  // write 0x80 <= 0x01
  localparam S_DATA_INIT_WAIT_WRITE_2      = 8'd11;
  localparam S_DATA_INIT_WRITE_3           = 8'd12;  // write 0xFF <= 0x01
  localparam S_DATA_INIT_WAIT_WRITE_3      = 8'd13;
  localparam S_DATA_INIT_WRITE_4           = 8'd14;  // write 0x00 <= 0x00
  localparam S_DATA_INIT_WAIT_WRITE_4      = 8'd15;
  localparam S_DATA_INIT_READ_1            = 8'd16;  // read reg 0x91 (stop_variable)
  localparam S_DATA_INIT_WAIT_READ_1       = 8'd17;
  localparam S_DATA_INIT_WRITE_5           = 8'd18;  // write 0x00 <= 0x01
  localparam S_DATA_INIT_WAIT_WRITE_5      = 8'd19;
  localparam S_DATA_INIT_WRITE_6           = 8'd20;  // write 0xFF <= 0x00
  localparam S_DATA_INIT_WAIT_WRITE_6      = 8'd21;
  localparam S_DATA_INIT_WRITE_7           = 8'd22;  // write 0x80 <= 0x00
  localparam S_DATA_INIT_WAIT_WRITE_7      = 8'd23;
  // --- LOAD DEFAULT TUNING SETTINGS (sử dụng ROM)
  localparam S_TUNING_INIT                = 8'd24;
  localparam S_TUNING_WRITE               = 8'd25;
  localparam S_TUNING_WAIT                = 8'd26;
  localparam S_TUNING_DONE                = 8'd27;
  // --- CONFIGURE INTERRUPT
  localparam S_CONFIG_INT_WRITE          = 8'd28;  // write reg 0x0A <= 0x04
  localparam S_CONFIG_INT_WAIT_WRITE     = 8'd29;
  localparam S_CONFIG_INT_READ           = 8'd30;  // read reg 0x84
  localparam S_CONFIG_INT_WAIT_READ      = 8'd31;
  localparam S_CONFIG_INT_WRITE2         = 8'd32;  // write reg 0x84 <= (read_val & ~0x10)
  localparam S_CONFIG_INT_WAIT_WRITE2    = 8'd33;
  localparam S_CONFIG_INT_WRITE3         = 8'd34;  // write reg 0x0B <= 0x01
  localparam S_CONFIG_INT_WAIT_WRITE3    = 8'd35;
  // --- SET SEQUENCE STEPS ENABLED
  localparam S_SET_SEQ_WRITE           = 8'd36;   // write reg 0x01 <= SEQUENCE_ENABLE
  localparam S_SET_SEQ_WAIT_WRITE      = 8'd37;
  // --- PERFORM CALIBRATION: VHV
  localparam S_CALIB_VHV_WRITE_SEQ      = 8'd38;   // write reg 0x01 <= 0x01
  localparam S_CALIB_VHV_WAIT_WRITE_SEQ = 8'd39;
  localparam S_CALIB_VHV_WRITE_SYSRANGE = 8'd40;   // write reg 0x00 <= 0x41 (0x01|0x40)
  localparam S_CALIB_VHV_WAIT_WRITE_SYSRANGE = 8'd41;
  localparam S_CALIB_VHV_WAIT_INTERRUPT = 8'd42;   // poll reg 0x13 cho interrupt !=0
  localparam S_CALIB_VHV_CLEAR_INT      = 8'd43;   // write reg 0x0B <= 0x01
  localparam S_CALIB_VHV_WAIT_CLEAR     = 8'd44;
  localparam S_CALIB_VHV_WRITE_SYSRANGE_CLEAR = 8'd45; // write reg 0x00 <= 0x00
  localparam S_CALIB_VHV_WAIT_WRITE_SYSRANGE_CLEAR = 8'd46;
  // --- PERFORM CALIBRATION: PHASE
  localparam S_CALIB_PHASE_WRITE_SEQ      = 8'd47;   // write reg 0x01 <= 0x02
  localparam S_CALIB_PHASE_WAIT_WRITE_SEQ = 8'd48;
  localparam S_CALIB_PHASE_WRITE_SYSRANGE = 8'd49;   // write reg 0x00 <= 0x01 (0x01|0x00)
  localparam S_CALIB_PHASE_WAIT_WRITE_SYSRANGE = 8'd50;
  localparam S_CALIB_PHASE_WAIT_INTERRUPT = 8'd51;   // poll reg 0x13
  localparam S_CALIB_PHASE_CLEAR_INT      = 8'd52;   // write reg 0x0B <= 0x01
  localparam S_CALIB_PHASE_WAIT_CLEAR     = 8'd53;
  localparam S_CALIB_PHASE_WRITE_SYSRANGE_CLEAR = 8'd54; // write reg 0x00 <= 0x00
  localparam S_CALIB_PHASE_WAIT_WRITE_SYSRANGE_CLEAR = 8'd55;
  // --- RE-ENABLE SEQUENCE STEPS
  localparam S_SET_SEQ_WRITE2           = 8'd56;  // write reg 0x01 <= SEQUENCE_ENABLE
  localparam S_SET_SEQ_WAIT_WRITE2      = 8'd57;
  // --- INIT DONE
  localparam S_INIT_DONE                = 8'd58;
  // --- READ RANGE SEQUENCE (chỉ thực hiện sau khi khởi tạo thành công)
  localparam S_READ_PREPARE_0           = 8'd59;   // write reg 0x80 <= 0x01
  localparam S_READ_WAIT_0              = 8'd60;
  localparam S_READ_PREPARE_1           = 8'd61;   // write reg 0xFF <= 0x01
  localparam S_READ_WAIT_1              = 8'd62;
  localparam S_READ_PREPARE_2           = 8'd63;   // write reg 0x00 <= 0x00
  localparam S_READ_WAIT_2              = 8'd64;
  localparam S_READ_PREPARE_3           = 8'd65;   // write reg 0x91 <= stop_variable
  localparam S_READ_WAIT_3              = 8'd66;
  localparam S_READ_PREPARE_4           = 8'd67;   // write reg 0x00 <= 0x01
  localparam S_READ_WAIT_4              = 8'd68;
  localparam S_READ_PREPARE_5           = 8'd69;   // write reg 0xFF <= 0x00
  localparam S_READ_WAIT_5              = 8'd70;
  localparam S_READ_PREPARE_6           = 8'd71;   // write reg 0x80 <= 0x00
  localparam S_READ_WAIT_6              = 8'd72;
  localparam S_READ_START               = 8'd73;   // write reg SYSRANGE_START (0x00) <= 0x01
  localparam S_READ_WAIT_SYSRANGE        = 8'd74;  // poll reg SYSRANGE_START (0x00) cho bit0 = 0
  localparam S_READ_WAIT_INTERRUPT       = 8'd75;  // poll reg RESULT_INTERRUPT_STATUS (0x13) cho giá trị != 0
  localparam S_READ_DATA                = 8'd76;   // đọc 16-bit range từ (REG_RESULT_RANGE_STATUS+10)
  localparam S_READ_CLEAR_INT           = 8'd77;   // write reg 0x0B <= 0x01
  localparam S_READ_DONE                = 8'd78;

  //-------------------------------------------------------------------------
  // Nội bộ: ROM chứa default tuning settings
  // Mỗi phần tử: [15:8]=địa chỉ, [7:0]=dữ liệu
  // Số phần tử = 80
  //-------------------------------------------------------------------------
  localparam TUNING_COUNT = 80;
  reg [15:0] tuning_rom [0:TUNING_COUNT-1];
  reg [6:0] tuning_index;  // chỉ số trong ROM (cần 7 bit cho 80 phần tử)

  // Khởi tạo ROM (sử dụng khối initial; trong syntheses bạn có thể sử dụng file external)
  integer i;
  initial begin
    // Chỉ liệt kê các giao dịch theo thứ tự trong code C
    tuning_rom[ 0] = {8'hFF, 8'h01};
    tuning_rom[ 1] = {8'h00, 8'h00};
    tuning_rom[ 2] = {8'hFF, 8'h00};
    tuning_rom[ 3] = {8'h09, 8'h00};
    tuning_rom[ 4] = {8'h10, 8'h00};
    tuning_rom[ 5] = {8'h11, 8'h00};
    tuning_rom[ 6] = {8'h24, 8'h01};
    tuning_rom[ 7] = {8'h25, 8'hFF};
    tuning_rom[ 8] = {8'h75, 8'h00};
    tuning_rom[ 9] = {8'hFF, 8'h01};
    tuning_rom[10] = {8'h4E, 8'h2C};
    tuning_rom[11] = {8'h48, 8'h00};
    tuning_rom[12] = {8'h30, 8'h20};
    tuning_rom[13] = {8'hFF, 8'h00};
    tuning_rom[14] = {8'h30, 8'h09};
    tuning_rom[15] = {8'h54, 8'h00};
    tuning_rom[16] = {8'h31, 8'h04};
    tuning_rom[17] = {8'h32, 8'h03};
    tuning_rom[18] = {8'h40, 8'h83};
    tuning_rom[19] = {8'h46, 8'h25};
    tuning_rom[20] = {8'h60, 8'h00};
    tuning_rom[21] = {8'h27, 8'h00};
    tuning_rom[22] = {8'h50, 8'h06};
    tuning_rom[23] = {8'h51, 8'h00};
    tuning_rom[24] = {8'h52, 8'h96};
    tuning_rom[25] = {8'h56, 8'h08};
    tuning_rom[26] = {8'h57, 8'h30};
    tuning_rom[27] = {8'h61, 8'h00};
    tuning_rom[28] = {8'h62, 8'h00};
    tuning_rom[29] = {8'h64, 8'h00};
    tuning_rom[30] = {8'h65, 8'h00};
    tuning_rom[31] = {8'h66, 8'hA0};
    tuning_rom[32] = {8'hFF, 8'h01};
    tuning_rom[33] = {8'h22, 8'h32};
    tuning_rom[34] = {8'h47, 8'h14};
    tuning_rom[35] = {8'h49, 8'hFF};
    tuning_rom[36] = {8'h4A, 8'h00};
    tuning_rom[37] = {8'hFF, 8'h00};
    tuning_rom[38] = {8'h7A, 8'h0A};
    tuning_rom[39] = {8'h7B, 8'h00};
    tuning_rom[40] = {8'h78, 8'h21};
    tuning_rom[41] = {8'hFF, 8'h01};
    tuning_rom[42] = {8'h23, 8'h34};
    tuning_rom[43] = {8'h42, 8'h00};
    tuning_rom[44] = {8'h44, 8'hFF};
    tuning_rom[45] = {8'h45, 8'h26};
    tuning_rom[46] = {8'h46, 8'h05};
    tuning_rom[47] = {8'h40, 8'h40};
    tuning_rom[48] = {8'h0E, 8'h06};
    tuning_rom[49] = {8'h20, 8'h1A};
    tuning_rom[50] = {8'h43, 8'h40};
    tuning_rom[51] = {8'hFF, 8'h00};
    tuning_rom[52] = {8'h34, 8'h03};
    tuning_rom[53] = {8'h35, 8'h44};
    tuning_rom[54] = {8'hFF, 8'h01};
    tuning_rom[55] = {8'h31, 8'h04};
    tuning_rom[56] = {8'h4B, 8'h09};
    tuning_rom[57] = {8'h4C, 8'h05};
    tuning_rom[58] = {8'h4D, 8'h04};
    tuning_rom[59] = {8'hFF, 8'h00};
    tuning_rom[60] = {8'h44, 8'h00};
    tuning_rom[61] = {8'h45, 8'h20};
    tuning_rom[62] = {8'h47, 8'h08};
    tuning_rom[63] = {8'h48, 8'h28};
    tuning_rom[64] = {8'h67, 8'h00};
    tuning_rom[65] = {8'h70, 8'h04};
    tuning_rom[66] = {8'h71, 8'h01};
    tuning_rom[67] = {8'h72, 8'hFE};
    tuning_rom[68] = {8'h76, 8'h00};
    tuning_rom[69] = {8'h77, 8'h00};
    tuning_rom[70] = {8'hFF, 8'h01};
    tuning_rom[71] = {8'h0D, 8'h01};
    tuning_rom[72] = {8'hFF, 8'h00};
    tuning_rom[73] = {8'h80, 8'h01};
    tuning_rom[74] = {8'h01, 8'hF8};
    tuning_rom[75] = {8'hFF, 8'h01};
    tuning_rom[76] = {8'h8E, 8'h01};
    tuning_rom[77] = {8'h00, 8'h01};
    tuning_rom[78] = {8'hFF, 8'h00};
    tuning_rom[79] = {8'h80, 8'h00};
  end

 //-------------------------------------------------------------------------
// FSM internal registers
//-------------------------------------------------------------------------
reg [7:0] state;
reg [7:0] device_id;
reg [7:0] temp_reg;   // lưu giá trị tạm thời (ví dụ: cho VHV config)
reg [7:0] stop_variable;  // lưu giá trị stop_variable từ data_init_read_1

//-------------------------------------------------------------------------
// FSM chính
//-------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state       <= S_IDLE;
    init_done   <= 1'b0;
    read_done   <= 1'b0;
    error       <= 1'b0;
    i2c_start   <= 1'b0;
    i2c_rw      <= 1'b0;
    i2c_reg_addr<= 8'd0;
    i2c_data_out<= 8'd0;
    tuning_index<= 0;
    range       <= 16'd0;
  end
  else begin
    // Mặc định tắt i2c_start, reset các flag done
    i2c_start <= 1'b0;
    case(state)
      //===========================
      // 1. INIT: Kiểm tra Device ID
      S_IDLE: begin
        init_done <= 1'b0;
        read_done <= 1'b0;
        error     <= 1'b0;
        if(start_init) begin
          state <= S_INIT_READ_DEVICE_ID;
        end
      end

      S_INIT_READ_DEVICE_ID: begin
        // Bắt đầu đọc Device ID từ reg 0xC0
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1; // read
        i2c_reg_addr <= REG_IDENTIFICATION_MODEL_ID;
        state        <= S_INIT_WAIT_DEVICE_ID;
      end
      S_INIT_WAIT_DEVICE_ID: begin
        if(i2c_done) begin
          state <= S_INIT_CHECK_DEVICE_ID;
        end
      end
      S_INIT_CHECK_DEVICE_ID: begin
        device_id <= i2c_data_in;
        if(i2c_data_in == VL53L0X_EXPECTED_DEVICE_ID)
          state <= S_DATA_INIT_READ_0;
        else
          state <= S_IDLE; // lỗi Device ID => quay lại idle (hoặc có thể đặt error=1)
      end

      //===========================
      // 2. DATA_INIT: thực hiện chuỗi các giao dịch khởi tạo
      // - Bước 1: Đọc reg 0x89, sau đó ghi lại với bit0 được set.
      S_DATA_INIT_READ_0: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1; // read
        i2c_reg_addr <= REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV;
        state        <= S_DATA_INIT_WAIT_READ_0;
      end
      S_DATA_INIT_WAIT_READ_0: begin
        if(i2c_done)
          state <= S_DATA_INIT_WRITE_0;
      end
      S_DATA_INIT_WRITE_0: begin
        temp_reg    <= i2c_data_in | 8'h01;
        i2c_start   <= 1'b1;
        i2c_rw      <= 1'b0; // write
        i2c_reg_addr<= REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV;
        i2c_data_out<= i2c_data_in | 8'h01;
        state       <= S_DATA_INIT_WAIT_WRITE_0;
      end
      S_DATA_INIT_WAIT_WRITE_0: begin
        if(i2c_done)
          state <= S_DATA_INIT_WRITE_1;
      end
      // write 0x88 <= 0x00
      S_DATA_INIT_WRITE_1: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h88;
        i2c_data_out <= 8'h00;
        state        <= S_DATA_INIT_WAIT_WRITE_1;
      end
      S_DATA_INIT_WAIT_WRITE_1: begin
        if(i2c_done)
          state <= S_DATA_INIT_WRITE_2;
      end
      // write 0x80 <= 0x01
      S_DATA_INIT_WRITE_2: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h80;
        i2c_data_out <= 8'h01;
        state        <= S_DATA_INIT_WAIT_WRITE_2;
      end
      S_DATA_INIT_WAIT_WRITE_2: begin
        if(i2c_done)
          state <= S_DATA_INIT_WRITE_3;
      end
      // write 0xFF <= 0x01
      S_DATA_INIT_WRITE_3: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'hFF;
        i2c_data_out <= 8'h01;
        state        <= S_DATA_INIT_WAIT_WRITE_3;
      end
      S_DATA_INIT_WAIT_WRITE_3: begin
        if(i2c_done)
          state <= S_DATA_INIT_WRITE_4;
      end
      // write 0x00 <= 0x00
      S_DATA_INIT_WRITE_4: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h00;
        i2c_data_out <= 8'h00;
        state        <= S_DATA_INIT_WAIT_WRITE_4;
      end
      S_DATA_INIT_WAIT_WRITE_4: begin
        if(i2c_done)
          state <= S_DATA_INIT_READ_1;
      end
      // read 0x91 (lưu vào stop_variable)
      S_DATA_INIT_READ_1: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1;
        i2c_reg_addr <= 8'h91;
        state        <= S_DATA_INIT_WAIT_READ_1;
      end
      S_DATA_INIT_WAIT_READ_1: begin
        if(i2c_done) begin
          stop_variable <= i2c_data_in;
          state <= S_DATA_INIT_WRITE_5;
        end
      end
      // write 0x00 <= 0x01
      S_DATA_INIT_WRITE_5: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h00;
        i2c_data_out <= 8'h01;
        state        <= S_DATA_INIT_WAIT_WRITE_5;
      end
      S_DATA_INIT_WAIT_WRITE_5: begin
        if(i2c_done)
          state <= S_DATA_INIT_WRITE_6;
      end
      // write 0xFF <= 0x00
      S_DATA_INIT_WRITE_6: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'hFF;
        i2c_data_out <= 8'h00;
        state        <= S_DATA_INIT_WAIT_WRITE_6;
      end
      S_DATA_INIT_WAIT_WRITE_6: begin
        if(i2c_done)
          state <= S_DATA_INIT_WRITE_7;
      end
      // write 0x80 <= 0x00
      S_DATA_INIT_WRITE_7: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h80;
        i2c_data_out <= 8'h00;
        state        <= S_DATA_INIT_WAIT_WRITE_7;
      end
      S_DATA_INIT_WAIT_WRITE_7: begin
        if(i2c_done)
          state <= S_TUNING_INIT;
      end

      //===========================
      // 3. LOAD DEFAULT TUNING SETTINGS (sử dụng ROM)
      S_TUNING_INIT: begin
        tuning_index <= 0;
        state <= S_TUNING_WRITE;
      end
      S_TUNING_WRITE: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= tuning_rom[tuning_index][15:8];
        i2c_data_out <= tuning_rom[tuning_index][7:0];
        state        <= S_TUNING_WAIT;
      end
      S_TUNING_WAIT: begin
        if(i2c_done) begin
          if(tuning_index == TUNING_COUNT-1)
            state <= S_TUNING_DONE;
          else begin
            tuning_index <= tuning_index + 1;
            state <= S_TUNING_WRITE;
          end
        end
      end
      S_TUNING_DONE: begin
        state <= S_CONFIG_INT_WRITE;
      end

      //===========================
      // 4. CONFIGURE INTERRUPT
      S_CONFIG_INT_WRITE: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_INTERRUPT_CONFIG_GPIO;
        i2c_data_out <= 8'h04;
        state        <= S_CONFIG_INT_WAIT_WRITE;
      end
      S_CONFIG_INT_WAIT_WRITE: begin
        if(i2c_done)
          state <= S_CONFIG_INT_READ;
      end
      S_CONFIG_INT_READ: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1;
        i2c_reg_addr <= REG_GPIO_HV_MUX_ACTIVE_HIGH;
        state        <= S_CONFIG_INT_WAIT_READ;
      end
      S_CONFIG_INT_WAIT_READ: begin
        if(i2c_done)
          state <= S_CONFIG_INT_WRITE2;
      end
      S_CONFIG_INT_WRITE2: begin
        // xóa bit 4: mask ~0x10
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_GPIO_HV_MUX_ACTIVE_HIGH;
        i2c_data_out <= i2c_data_in & ~8'h10;
        state        <= S_CONFIG_INT_WAIT_WRITE2;
      end
      S_CONFIG_INT_WAIT_WRITE2: begin
        if(i2c_done)
          state <= S_CONFIG_INT_WRITE3;
      end
      S_CONFIG_INT_WRITE3: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_INTERRUPT_CLEAR;
        i2c_data_out <= 8'h01;
        state        <= S_CONFIG_INT_WAIT_WRITE3;
      end
      S_CONFIG_INT_WAIT_WRITE3: begin
        if(i2c_done)
          state <= S_SET_SEQ_WRITE;
      end

      //===========================
      // 5. SET SEQUENCE STEPS ENABLED
      S_SET_SEQ_WRITE: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_SEQUENCE_CONFIG;
        i2c_data_out <= SEQUENCE_ENABLE;
        state        <= S_SET_SEQ_WAIT_WRITE;
      end
      S_SET_SEQ_WAIT_WRITE: begin
        if(i2c_done)
          state <= S_CALIB_VHV_WRITE_SEQ;
      end

      //===========================
      // 6. PERFORM CALIBRATION - VHV
      S_CALIB_VHV_WRITE_SEQ: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_SEQUENCE_CONFIG;
        i2c_data_out <= 8'h01; // CALIBRATION_TYPE_VHV
        state        <= S_CALIB_VHV_WAIT_WRITE_SEQ;
      end
      S_CALIB_VHV_WAIT_WRITE_SEQ: begin
        if(i2c_done)
          state <= S_CALIB_VHV_WRITE_SYSRANGE;
      end
      S_CALIB_VHV_WRITE_SYSRANGE: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSRANGE_START;
        i2c_data_out <= 8'h41; // 0x01|0x40
        state        <= S_CALIB_VHV_WAIT_WRITE_SYSRANGE;
      end
      S_CALIB_VHV_WAIT_WRITE_SYSRANGE: begin
        if(i2c_done)
          state <= S_CALIB_VHV_WAIT_INTERRUPT;
      end
      S_CALIB_VHV_WAIT_INTERRUPT: begin
        // Polling: đọc reg 0x13 cho đến khi (data_in & 0x07) != 0
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1;
        i2c_reg_addr <= REG_RESULT_INTERRUPT_STATUS;
        if(i2c_done) begin
          if(i2c_data_in[2:0] != 3'b000)
            state <= S_CALIB_VHV_CLEAR_INT;
          else
            state <= S_CALIB_VHV_WAIT_INTERRUPT; // tiếp tục polling
        end
      end
      S_CALIB_VHV_CLEAR_INT: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_INTERRUPT_CLEAR;
        i2c_data_out <= 8'h01;
        state        <= S_CALIB_VHV_WAIT_CLEAR;
      end
      S_CALIB_VHV_WAIT_CLEAR: begin
        if(i2c_done)
          state <= S_CALIB_VHV_WRITE_SYSRANGE_CLEAR;
      end
      S_CALIB_VHV_WRITE_SYSRANGE_CLEAR: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSRANGE_START;
        i2c_data_out <= 8'h00;
        state        <= S_CALIB_VHV_WAIT_WRITE_SYSRANGE_CLEAR;
      end
      S_CALIB_VHV_WAIT_WRITE_SYSRANGE_CLEAR: begin
        if(i2c_done)
          state <= S_CALIB_PHASE_WRITE_SEQ;
      end

      //===========================
      // 7. PERFORM CALIBRATION - PHASE
      S_CALIB_PHASE_WRITE_SEQ: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_SEQUENCE_CONFIG;
        i2c_data_out <= 8'h02; // CALIBRATION_TYPE_PHASE
        state        <= S_CALIB_PHASE_WAIT_WRITE_SEQ;
      end
      S_CALIB_PHASE_WAIT_WRITE_SEQ: begin
        if(i2c_done)
          state <= S_CALIB_PHASE_WRITE_SYSRANGE;
      end
      S_CALIB_PHASE_WRITE_SYSRANGE: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSRANGE_START;
        i2c_data_out <= 8'h01; // 0x01|0x00
        state        <= S_CALIB_PHASE_WAIT_WRITE_SYSRANGE;
      end
      S_CALIB_PHASE_WAIT_WRITE_SYSRANGE: begin
        if(i2c_done)
          state <= S_CALIB_PHASE_WAIT_INTERRUPT;
      end
      S_CALIB_PHASE_WAIT_INTERRUPT: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1;
        i2c_reg_addr <= REG_RESULT_INTERRUPT_STATUS;
        if(i2c_done) begin
          if(i2c_data_in[2:0] != 3'b000)
            state <= S_CALIB_PHASE_CLEAR_INT;
          else
            state <= S_CALIB_PHASE_WAIT_INTERRUPT;
        end
      end
      S_CALIB_PHASE_CLEAR_INT: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_INTERRUPT_CLEAR;
        i2c_data_out <= 8'h01;
        state        <= S_CALIB_PHASE_WAIT_CLEAR;
      end
      S_CALIB_PHASE_WAIT_CLEAR: begin
        if(i2c_done)
          state <= S_CALIB_PHASE_WRITE_SYSRANGE_CLEAR;
      end
      S_CALIB_PHASE_WRITE_SYSRANGE_CLEAR: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSRANGE_START;
        i2c_data_out <= 8'h00;
        state        <= S_CALIB_PHASE_WAIT_WRITE_SYSRANGE_CLEAR;
      end
      S_CALIB_PHASE_WAIT_WRITE_SYSRANGE_CLEAR: begin
        if(i2c_done)
          state <= S_SET_SEQ_WRITE2;
      end

      //===========================
      // 8. Re-enable sequence steps
      S_SET_SEQ_WRITE2: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSTEM_SEQUENCE_CONFIG;
        i2c_data_out <= SEQUENCE_ENABLE;
        state        <= S_SET_SEQ_WAIT_WRITE2;
      end
      S_SET_SEQ_WAIT_WRITE2: begin
        if(i2c_done)
          state <= S_INIT_DONE;
      end

      //===========================
      // 9. INIT DONE
      S_INIT_DONE: begin
        init_done <= 1'b1;
        // Sau khi init xong, chờ lệnh start_read để chuyển sang chế độ đo
        if(start_read)
          state <= S_READ_PREPARE_0;
      end

      //===========================
      // 10. READ RANGE SEQUENCE
      S_READ_PREPARE_0: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h80;
        i2c_data_out <= 8'h01;
        state        <= S_READ_WAIT_0;
      end
      S_READ_WAIT_0: begin
        if(i2c_done)
          state <= S_READ_PREPARE_1;
      end
      S_READ_PREPARE_1: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'hFF;
        i2c_data_out <= 8'h01;
        state        <= S_READ_WAIT_1;
      end
      S_READ_WAIT_1: begin
        if(i2c_done)
          state <= S_READ_PREPARE_2;
      end
      S_READ_PREPARE_2: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h00;
        i2c_data_out <= 8'h00;
        state        <= S_READ_WAIT_2;
      end
      S_READ_WAIT_2: begin
        if(i2c_done)
          state <= S_READ_PREPARE_3;
      end
      S_READ_PREPARE_3: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h91;
        i2c_data_out <= stop_variable;
        state        <= S_READ_WAIT_3;
      end
      S_READ_WAIT_3: begin
        if(i2c_done)
          state <= S_READ_PREPARE_4;
      end
      S_READ_PREPARE_4: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h00;
        i2c_data_out <= 8'h01;
        state        <= S_READ_WAIT_4;
      end
      S_READ_WAIT_4: begin
        if(i2c_done)
          state <= S_READ_PREPARE_5;
      end
      S_READ_PREPARE_5: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'hFF;
        i2c_data_out <= 8'h00;
        state        <= S_READ_WAIT_5;
      end
      S_READ_WAIT_5: begin
        if(i2c_done)
          state <= S_READ_PREPARE_6;
      end
      S_READ_PREPARE_6: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= 8'h80;
        i2c_data_out <= 8'h00;
        state        <= S_READ_WAIT_6;
      end
      S_READ_WAIT_6: begin
        if(i2c_done)
          state <= S_READ_START;
      end
      S_READ_START: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b0;
        i2c_reg_addr <= REG_SYSRANGE_START;
        i2c_data_out <= 8'h01;
        state        <= S_READ_WAIT_SYSRANGE;
      end
      S_READ_WAIT_SYSRANGE: begin
        // Polling: đọc reg SYSRANGE_START cho đến khi bit0 = 0
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1;
        i2c_reg_addr <= REG_SYSRANGE_START;
        if(i2c_done) begin
          if((i2c_data_in & 8'h01) == 8'h00)
            state <= S_READ_WAIT_INTERRUPT;
          else
            state <= S_READ_WAIT_SYSRANGE;
        end
      end
      S_READ_WAIT_INTERRUPT: begin
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1;
        i2c_reg_addr <= REG_RESULT_INTERRUPT_STATUS;
        if(i2c_done) begin
          if(i2c_data_in[2:0] != 3'b000)
            state <= S_READ_DATA;
          else
            state <= S_READ_WAIT_INTERRUPT;
        end
      end
      S_READ_DATA: begin
        // Ở code C, đọc 16-bit range từ (REG_RESULT_RANGE_STATUS+10)
        // Giả sử địa chỉ bắt đầu = (0x14 + 10) = 8'h1E
        i2c_start    <= 1'b1;
        i2c_rw       <= 1'b1;
        i2c_reg_addr <= 8'h1E; // địa chỉ bắt đầu đọc 16-bit
        state        <= S_READ_CLEAR_INT;
      end
      S_READ_CLEAR_INT: begin
        if(i2c_done) begin
          // Giả sử i2c_data_in chứa phần high của range, và sau đó hệ thống đọc phần low ở lần giao dịch kế (để đơn giản, ta gộp thành 16-bit)
          range <= {i2c_data_in, 8'd0}; // ví dụ: đọc high byte, low byte = 0; trong thực tế cần giao dịch 2 byte
          i2c_start    <= 1'b1;
          i2c_rw       <= 1'b0;
          i2c_reg_addr <= REG_SYSTEM_INTERRUPT_CLEAR;
          i2c_data_out <= 8'h01;
          state        <= S_READ_DONE;
        end
      end
      S_READ_DONE: begin
        if(i2c_done) begin
          // Kiểm tra giá trị range: nếu 8190 hoặc 8191, báo hiệu out-of-range
          if(range == 16'd8190 || range == 16'd8191)
            range <= VL53L0X_OUT_OF_RANGE;
          read_done <= 1'b1;
          state <= S_IDLE; // quay lại idle, sẵn sàng cho giao dịch tiếp theo
        end
      end

      default: state <= S_IDLE;
    endcase
  end
end

endmodule

