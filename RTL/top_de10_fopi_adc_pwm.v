`timescale 1ns/1ps

module top_de10_fopi_adc_pwm (
    input  wire        CLOCK_50,
    input  wire        RESET_N,

    // ===== ADC SPI =====
    output wire        ADC_SCLK,
    output wire        ADC_CS_N,
    output wire        ADC_SDI,
    input  wire        ADC_SDO,

    // ===== PWM OUTPUTS =====
    output wire        PWM_HIGH,
    output wire        PWM_LOW,

    // ===== DEBUG =====
    output wire [9:0]  DUTY_OUT,
    output wire [31:0] VOUT_MV
);

    /* ---------------- Internal ---------------- */
    wire rst = ~RESET_N;

    wire [11:0] adc_data;
    wire        adc_ready;

    wire [31:0] vout_mV;
    wire        control_tick;
    wire [9:0]  duty_fopi;
    wire [9:0]  duty_pwm;

    /* ---------------- ADC SPI ---------------- */
    adc_spi_interface u_adc (
        .clk        (CLOCK_50),
        .rst_n      (RESET_N),
        .channel    (3'd0),      // ADC channel 0
        .start      (1'b1),       // continuous sampling
        .adc_sclk   (ADC_SCLK),
        .adc_cs_n   (ADC_CS_N),
        .adc_sdi    (ADC_SDI),
        .adc_sdo    (ADC_SDO),
        .data       (adc_data),
        .data_ready (adc_ready)
    );

    /* -------- Voltage Scaling -------- */
    voltage_scale u_scale (
        .clk        (CLOCK_50),
        .rst_n      (RESET_N),
        .adc_data   (adc_data),
        .data_ready (adc_ready),
        .vout_mV    (vout_mV),
        .vout_valid ()
    );

    /* -------- Control Tick (10 kHz) -------- */
    control_tick_gen #(
        .CLK_FREQ  (50_000_000),
        .CTRL_FREQ (10_000)
    ) u_tick (
        .clk          (CLOCK_50),
        .rst          (rst),
        .control_tick (control_tick)
    );

    /* -------- FOPI Controller -------- */
    fopi_controller_55V u_fopi (
        .clk          (CLOCK_50),
        .rst          (rst),
        .control_tick (control_tick),
        .vref_mV      (16'd55000),     // target = 55 V
        .vout_mV      (vout_mV[15:0]),
        .duty         (duty_fopi)
    );

    /* -------- Duty Latch -------- */
    duty_latch #(.DUTY_WIDTH(10)) u_latch (
        .clk          (CLOCK_50),
        .rst          (rst),
        .control_tick (control_tick),
        .duty_in      (duty_fopi),
        .duty_out     (duty_pwm)
    );

    /* -------- PWM + Dead-Time -------- */
    pwm_deadtime_2mosfet u_pwm (
        .clk      (CLOCK_50),
        .rst      (rst),
        .duty     (duty_pwm),
        .pwm_high (PWM_HIGH),
        .pwm_low  (PWM_LOW)
    );

    assign DUTY_OUT = duty_pwm;
    assign VOUT_MV  = vout_mV;

endmodule


// ============================================
// ADC SPI INTERFACE 
// ============================================

module adc_spi_interface (
    input        clk,          
    input        rst_n,        
    input [2:0]  channel,      
    input        start,        
    output reg   adc_sclk,     
    output reg   adc_cs_n,     
    output reg   adc_sdi,      
    input        adc_sdo,      
    output reg [11:0] data,    
    output reg   data_ready    
);

    localparam IDLE      = 3'd0;
    localparam SEND_CMD  = 3'd1;
    localparam READ_DATA = 3'd2;
    localparam FINISH    = 3'd3;

    reg [2:0] state;
    reg [5:0] bit_cnt;
    reg [11:0] shift_reg;
    reg [7:0] sclk_cnt;
    reg sclk_en;
    reg prev_sclk;
    reg [5:0] cmd_reg;
    
    localparam SCLK_DIV = 10;

    // SCLK Generation
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_cnt <= 0;
            adc_sclk <= 0;
        end else begin
            if (sclk_en) begin
                if (sclk_cnt == SCLK_DIV - 1) begin
                    sclk_cnt <= 0;
                    adc_sclk <= ~adc_sclk;
                end else begin
                    sclk_cnt <= sclk_cnt + 1;
                end
            end else begin
                adc_sclk <= 0;
                sclk_cnt <= 0;
            end
        end
    end

    // Main FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            adc_cs_n <= 1'b1;
            adc_sdi <= 1'b0;
            data_ready <= 1'b0;
            bit_cnt <= 6'd0;
            sclk_en <= 1'b0;
            prev_sclk <= 1'b0;
            cmd_reg <= 6'd0;
            shift_reg <= 12'd0;
            data <= 12'd0;
        end else begin
            prev_sclk <= adc_sclk;
            data_ready <= 1'b0;

            case (state)
                IDLE: begin
                    adc_cs_n <= 1'b1;
                    sclk_en <= 1'b0;
                    bit_cnt <= 6'd0;
                    
                    if (start) begin
                        state <= SEND_CMD;
                        adc_cs_n <= 1'b0;
                        sclk_en <= 1'b1;
                        bit_cnt <= 6'd0;
                        cmd_reg <= {1'b1, channel[0], channel[2:1], 1'b1, 1'b0};
                    end
                end

                SEND_CMD: begin
                    if ((prev_sclk == 1'b0) && (adc_sclk == 1'b1)) begin
                        if (bit_cnt < 6) begin
                            adc_sdi <= cmd_reg[5 - bit_cnt];
                            bit_cnt <= bit_cnt + 1'b1;
                        end else begin
                            state <= READ_DATA;
                            bit_cnt <= 6'd0;
                            shift_reg <= 12'd0;
                        end
                    end
                end

                READ_DATA: begin
                    if ((prev_sclk == 1'b0) && (adc_sclk == 1'b1)) begin
                        // SHIFT LEFT - Receive MSB first
                        shift_reg <= {shift_reg[10:0], adc_sdo};
                        
                        if (bit_cnt < 11) begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end else begin
                            state <= FINISH;
                        end
                    end
                end

                FINISH: begin
                    adc_cs_n <= 1'b1;
                    sclk_en <= 1'b0;
                    data <= shift_reg;
                    data_ready <= 1'b1;
                    state <= IDLE;
                end

                default: 
                    state <= IDLE;
            endcase
        end
    end

endmodule

// ============================================
// VOLTAGE SCALE
// ============================================

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

// ============================================
// CONTROL TICK GEN
// ============================================

module control_tick_gen #(
    parameter CLK_FREQ = 100_000_000,
    parameter CTRL_FREQ = 10_000
)(
    input  wire clk,
    input  wire rst,
    output reg  control_tick
);

    localparam integer DIV = CLK_FREQ / CTRL_FREQ;
    integer cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt <= 0;
            control_tick <= 0;
        end else begin
            if (cnt == DIV-1) begin
                cnt <= 0;
                control_tick <= 1;   // one-cycle pulse
            end else begin
                cnt <= cnt + 1;
                control_tick <= 0;
            end
        end
    end
endmodule

// ============================================
// FOPI CONTROLLER
// ============================================

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

// ============================================
// DUTY LATCH
// ============================================

module duty_latch #(
    parameter DUTY_WIDTH = 12
)(
    input  wire clk,
    input  wire rst,
    input  wire control_tick,
    input  wire [DUTY_WIDTH-1:0] duty_in,
    output reg  [DUTY_WIDTH-1:0] duty_out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            duty_out <= 0;
        end else if (control_tick) begin
            duty_out <= duty_in;  // update ONLY at control rate
        end
    end
endmodule

// ============================================
// PWM DEADTIME 
// ============================================

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
