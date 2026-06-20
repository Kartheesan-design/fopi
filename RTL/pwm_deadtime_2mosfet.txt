module pwm_deadtime_2mosfet (
    input  wire        clk,
    input  wire        rst,
    input  wire [9:0]  duty,      // 0–1000
    output reg         pwm_high,  // MOSFET QH
    output reg         pwm_low    // MOSFET QL
);

    parameter integer PWM_PERIOD = 1000;
    parameter integer DEAD_TIME  = 20;

    reg [9:0] pwm_cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pwm_cnt  <= 0;
            pwm_high <= 0;
            pwm_low  <= 0;
        end else begin
            // PWM counter
            if (pwm_cnt == PWM_PERIOD-1)
                pwm_cnt <= 0;
            else
                pwm_cnt <= pwm_cnt + 1;

            // HIGH-side MOSFET
            if (pwm_cnt < duty - DEAD_TIME)
                pwm_high <= 1;
            else
                pwm_high <= 0;

            // LOW-side MOSFET (complementary with dead-time)
            if (pwm_cnt > duty + DEAD_TIME)
                pwm_low <= 1;
            else
                pwm_low <= 0;
        end
    end
endmodule
