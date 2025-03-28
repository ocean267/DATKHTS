module clk_divider #(
    parameter   input_clk_freq  = 100_000_000,      // Input clock 100 MHz
    parameter   output_clk_freq =   1_000_000       // Output clock  1 MHz = 1 us
)(
    input       clk,
    input       rst_n,          // Thêm tín hiệu reset (active low)
    output      clk_1MHz                                   
);
    reg         clk_1MHz_temp;
    reg [31:0]  count;
    localparam  half_cycle = input_clk_freq / output_clk_freq / 2 - 1;
    
    // Khởi tạo lại các giá trị khi reset
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_1MHz_temp <= 1'b0;
            count <= 0;
        end else begin
            if (count == half_cycle) begin
                clk_1MHz_temp <= ~clk_1MHz_temp;
                count <= 0;
            end else begin
                count <= count + 1;
            end
        end
    end

    assign clk_1MHz = clk_1MHz_temp;
    
endmodule
