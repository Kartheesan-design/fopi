// ============================================
// ADC SPI INTERFACE - COMPLETELY CORRECTED
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
