// Code your design here
module voltage_scale (
    input        clk,
    input        rst_n,
    input [11:0] adc_data,
    input        data_ready,
    output reg [31:0] vout_mV,
    output reg        vout_valid
);

    // Constants
    localparam integer VREF_MV = 3000;
    localparam integer GAIN    = 28;
    localparam integer ADC_MAX = 4095;

    reg [31:0] mult_result;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vout_mV    <= 32'd0;
            vout_valid <= 1'b0;
            mult_result <= 32'd0;
        end else begin
            vout_valid <= 1'b0;

            if (data_ready) begin
                // Multiply first (safe width)
                mult_result <= adc_data * VREF_MV * GAIN;

                // Divide last
                vout_mV <= mult_result / ADC_MAX;

                vout_valid <= 1'b1;
            end
        end
    end

endmodule
