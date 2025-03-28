module sensor_auto_ctrl(
    input  wire clk,
    input  wire rst_n,
    input  wire init_done,
    input  wire read_done,
    output reg  start_init,
    output reg  start_read
);

    // Định nghĩa trạng thái bằng parameter
    parameter IDLE = 2'b00;
    parameter INIT = 2'b01;
    parameter READ = 2'b10;

    reg [1:0] state, next_state;
    reg [23:0] counter;
    localparam DELAY = 24'd10_000_000;  // Điều chỉnh thời gian chờ theo clock

    // FSM: Cập nhật trạng thái và bộ đếm
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            counter <= 24'd0;
        end else begin
            state <= next_state;
            // Nếu ở trạng thái IDLE, tăng counter; ngược lại reset counter
            if (state == IDLE)
                counter <= counter + 1;
            else
                counter <= 24'd0;
        end
    end

    // FSM: Điều khiển chuyển trạng thái và tín hiệu start
    always @(*) begin
        // Mặc định các tín hiệu off
        next_state = state;
        start_init = 1'b0;
        start_read = 1'b0;

        case (state)
            IDLE: begin
                if (counter >= DELAY)
                    next_state = INIT;
            end

            INIT: begin
                start_init = 1'b1;
                if (init_done)
                    next_state = READ;
            end

            READ: begin
                start_read = 1'b1;
                if (read_done)
                    next_state = IDLE;  // Sau khi đọc xong, quay lại IDLE để lặp lại chu kỳ
            end

            default: next_state = IDLE;
        endcase
    end

endmodule
