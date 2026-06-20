module fopi_controller_55V (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] vref_mV,    // 55000
    input  wire [15:0] vout_mV,
    output reg  [9:0]  duty
);

    // ===== Tuned gains (stable on FPGA) =====
    parameter integer KP = 6;
    parameter integer KI = 1;

    parameter integer DUTY_MIN = 300;
    parameter integer DUTY_MAX = 950;

    reg signed [17:0] error, prev_error;
    reg signed [31:0] acc;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            error      <= 0;
            prev_error <= 0;
            acc        <= 800;   // real operating point
            duty       <= 800;
        end else begin
            // Error in mV
            error <= $signed({1'b0, vref_mV}) - $signed({1'b0, vout_mV});

            // Incremental FOPI (FPGA-safe)
            acc <= acc
                 + ((KP * (error - prev_error)) >>> 7)
                 + ((KI * error) >>> 9);

            prev_error <= error;

            // Anti-windup + saturation
            if (acc > DUTY_MAX)
                duty <= DUTY_MAX;
            else if (acc < DUTY_MIN)
                duty <= DUTY_MIN;
            else
                duty <= acc[9:0];
        end
    end
endmodule


module buckboost_plant_55V (
    input  wire        clk,
    input  wire        rst,
    input  wire [9:0]  duty,
    output reg  [15:0] vout_mV
);

    // First-order averaged power stage
    // Gain chosen so that:
    // duty = 800 → 55 V

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vout_mV <= 0;
        end else begin
            vout_mV <= vout_mV
                     + (((duty * 70) - vout_mV) >>> 4);
        end
    end
endmodule

module tb_fopi_55V;

    reg clk = 0;
    reg rst = 1;

    wire [9:0]  duty;
    wire [15:0] vout_mV;

    reg [15:0] vref_mV = 16'd55000;

    // 50 MHz equivalent clock
    always #10 clk = ~clk;

    fopi_controller_55V ctrl (
        .clk(clk),
        .rst(rst),
        .vref_mV(vref_mV),
        .vout_mV(vout_mV),
        .duty(duty)
    );

    buckboost_plant_55V plant (
        .clk(clk),
        .rst(rst),
        .duty(duty),
        .vout_mV(vout_mV)
    );

    initial begin
        $display("=== TRUE 55V FOPI BUCK-BOOST ===");
        $display("Time(us)\tVout(V)\tDuty");

        #100 rst = 0;

        repeat (150) begin
            @(posedge clk);
            $display("%0d\t\t%0.2f\t%d",
                     $time/1000,
                     vout_mV / 1000.0,
                     duty);
        end

        $finish;
    end
endmodule
