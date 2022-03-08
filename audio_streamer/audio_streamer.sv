module AudioStreamer(
        input CLOCK_50, input CLOCK2_50, input [3:0] KEY, input [9:0] SW,
        input AUD_DACLRCK, input AUD_ADCLRCK, input AUD_BCLK, input AUD_ADCDAT,
        inout FPGA_I2C_SDAT, output FPGA_I2C_SCLK, output AUD_DACDAT, output AUD_XCK,
        output [6:0] HEX0, output [6:0] HEX1, output [6:0] HEX2,
        output [6:0] HEX3, output [6:0] HEX4, output [6:0] HEX5,
        output [9:0] LEDR);

    // define enumerated type for states
    typedef enum {
                state_read_sample,
                state_capture_sample,
                state_increment,
                state_wait_until_ready,
                state_send_sample,
                state_wait_for_accepted} state;

    state next_state;

    // define signals for sample rate
    reg slow_rate, fast_rate;
    // define signal to repeat every other sample for slow rate
    reg rep_samp;

    // switch logic to control sample rate
    always_comb
    begin
        case(SW[1:0])
            2'b00, 2'b11:
            begin
                slow_rate = 0;
                fast_rate = 0;
            end
            2'b01:
            begin
                slow_rate = 0;
                fast_rate = 1;
            end
            2'b10:
            begin
                slow_rate = 1;
                fast_rate = 0;
            end
            default:
            begin
                slow_rate = 0;
                fast_rate = 0;
            end
        endcase
    end

    // codec, config, and clock generator require active high reset
    wire rst;
    assign rst = ~(KEY[3]);

    clock_generator generator(CLOCK2_50, rst, AUD_XCK);
    audio_and_video_config config(CLOCK_50, rst, FPGA_I2C_SDAT, FPGA_I2C_SCLK);

    // signals for codec
    reg read_s, read_ready, write_ready, write_s;
    reg [15:0] writedata_left, writedata_right;
    reg [15:0] readdata_left, readdata_right;

    // microphone is not required
    assign read_s = 1'b0;

    audio_codec codec(
                    CLOCK_50,
                    rst,
                    read_s,
                    write_s,
                    writedata_left, writedata_right,
                    AUD_ADCDAT,
                    AUD_BCLK,
                    AUD_ADCLRCK,
                    AUD_DACLRCK,
                    read_ready,
                    write_ready,
                    readdata_left, readdata_right,
                    AUD_DACDAT);

    // signals for flash
    logic flash_mem_read;
    logic flash_mem_waitrequest;
    logic [22:0] flash_mem_address;
    logic [31:0] flash_mem_readdata;
    logic flash_mem_readdatavalid;
    logic [3:0] flash_mem_byteenable = 4'b1111;
    logic rst_n, clk;

    // flash requires active low reset
    assign rst_n = KEY[3];
    assign clk = CLOCK_50;

    flash flash(
              .clk_clk(clk),
              .reset_reset_n(rst_n),
              .flash_mem_write(1'b0),
              .flash_mem_burstcount(6'b1),
              .flash_mem_waitrequest(flash_mem_waitrequest),
              .flash_mem_read(flash_mem_read),
              .flash_mem_address(flash_mem_address),
              .flash_mem_readdata(flash_mem_readdata),
              .flash_mem_readdatavalid(flash_mem_readdatavalid),
              .flash_mem_byteenable(flash_mem_byteenable),
              .flash_mem_writedata());

    // signal to for delay timer
    integer delay_timer;
    // signal for 2 samples
    logic [31:0] audio_sample;
    // signal for each sample
    logic signed [15:0] mono_sample;
    // signal to switch between each 2B sample
    logic lor;

    // FSM controller
    always_ff @(posedge clk or negedge rst_n)
    begin
        if (rst_n == 1'b0)
        begin
            next_state <= state_read_sample;
            flash_mem_address <= 23'b0;
            flash_mem_read <= 0;
            write_s <= 0;
            delay_timer <= 0;
            rep_samp <= 0;

        end
        else
        begin
            case(next_state)
                state_read_sample:
                begin
                    if (~flash_mem_readdatavalid)
                    begin
                        next_state <= state_read_sample;
                    end
                    else
                    begin
                        next_state <= state_capture_sample;
                        flash_mem_read <= 0;
                    end
                    flash_mem_read <= 1;
                end

                state_capture_sample:
                begin
                    next_state <= state_increment;
                    audio_sample <= flash_mem_readdata;
                end

                state_increment:
                begin
                    // increment by 2 since 2 samples per cycle
                    delay_timer <= delay_timer + 2;
                    lor <= 0;
                    // read all 0x200000 samples in flash
                    if (delay_timer > 2097152)
                    begin
                        next_state <= state_read_sample;
                        flash_mem_address <= 23'b0;
                        flash_mem_read <= 0;
                        write_s <= 0;
                        delay_timer <= 0;
                        rep_samp <= 0;
                    end
                    else
                    begin
                        next_state <= state_wait_until_ready;
                        if (slow_rate)
                        begin
                            // repeat every other sample for slow rate
                            flash_mem_address <= flash_mem_address + rep_samp;
                            rep_samp <= ~rep_samp;
                        end
                        else if (fast_rate)
                            // skip every other sample for fast rate
                            flash_mem_address <= flash_mem_address + 2;
                        else
                            flash_mem_address <= flash_mem_address + 1;
                    end
                end

                state_wait_until_ready:
                begin
                    write_s <= 0;
                    if (write_ready)
                        next_state <= state_send_sample;
                end

                state_send_sample:
                begin
                    next_state <= state_wait_for_accepted;

                    if (~lor)
                        mono_sample = audio_sample[31:16];
                end
                else
                    mono_sample = audio_sample[15:0];

                writedata_left <= (mono_sample/64);
                writedata_right <= (mono_sample/64);
                write_s <= 1;
            end

            state_wait_for_accepted:
            begin

                if (~write_ready)
                begin
                    if (~lor)
                    begin
                        next_state <= state_wait_until_ready;
                        lor = 1;
                    end
                    else
                    begin
                        next_state <= state_read_sample;
                    end
                end
            end

            default:
            begin
                next_state <= state_read_sample;
            end

        endcase
    end
end
endmodule: AudioStreamer
