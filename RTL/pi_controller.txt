module pi_controller (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] vout_mv,
    output reg  [15:0] duty
);

    // Constants
    localparam signed [16:0] VREF = 17'sd56000;
    localparam integer DUTY_MIN = 0;
    localparam integer DUTY_MAX = 1000;
    localparam integer DUTY_MID = 500;

    localparam integer KP = 1;
    localparam integer KI = 1;
    localparam integer SCALE = 200;   
    // Signals
    wire signed [16:0] error;
    wire signed [31:0] p_term;
    wire signed [31:0] i_next;
    wire signed [31:0] pi_out;
    wire signed [31:0] duty_raw;

    reg  signed [31:0] i_term;

    // Combinational math (NO latency)
    assign error   = VREF - vout_mv;
    assign p_term  = (KP * error) / SCALE;
    assign i_next  = i_term + (KI * error) / SCALE;
    assign pi_out  = p_term + i_term;
    assign duty_raw = DUTY_MID + pi_out;

    // Sequential part
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            i_term <= 0;
            duty   <= DUTY_MID;
        end else begin
            // Clamp duty
            if (duty_raw > DUTY_MAX)
                duty <= DUTY_MAX;
            else if (duty_raw < DUTY_MIN)
                duty <= DUTY_MIN;
            else
                duty <= duty_raw[15:0];

            // Anti-windup
            if (duty > DUTY_MIN && duty < DUTY_MAX)
                i_term <= i_next;
            else
                i_term <= i_term;
        end
    end

endmodule
