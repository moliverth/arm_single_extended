module timer(input logic clk,
             input logic enable_clock,
             input logic reset_timestamp, 
             input logic reset_timer,
             input logic [63:0] timer_input,  // time to alarm in nano seconds
             output logic warn,
             output logic [63:0] timestamp);

  
  parameter real device_clk_frequency = 1e6; // 1 Mhz.
  parameter real device_clk_period = (1e12/device_clk_frequency); // 1e12 correspound to 1 sec in real datatype scale.
  parameter real timestamp_unit = 1e-6; // make 1 unit of timestamp correspond to 1us.
  real real_timestamp; // real datatype timestemp type.

  logic device_clk = 0; // initiate in low state.
  real  final_timer_timestamp;

  realclock realclock (.clk(device_clk),
                      .clk_frequency(device_clk_frequency),
                      .timestamp_unit(timestamp_unit),
                      .reset(reset_timestamp),
                      .enable(enable_clock),
                      .timestamp(real_timestamp));

  task initialize_timer(); begin
    final_timer_timestamp = $itor(timestamp + timer_input);
    warn = 0;
  end
  endtask

  task check_timer(); begin
    // run loop
    if (real_timestamp >= final_timer_timestamp) begin 
      warn = 1;
    end else 
      warn = 0;
  end
  endtask

  // generate device clk.
  always begin
    #(device_clk_period/2) device_clk <= ~device_clk;
  end

  // main loop. uses clock from cpu.
  always @ (negedge clk) begin
    assign timestamp = $rtoi(real_timestamp);
    if ((final_timer_timestamp != 0) & (timestamp != 0)) 
      check_timer();
    else
      warn = 0;
  end

  // reset timer
  always @ (posedge reset_timer) begin
    if (timer_input) initialize_timer();
    else begin 
      warn = 0;
      final_timer_timestamp = 0;
    end  
  end

  // start timer with timer input change
  always @ (timer_input) begin
    if (timer_input) initialize_timer();
    else begin 
      warn = 0;
      final_timer_timestamp = 0;
    end  
  end

  // reset timestamp
  always @ (posedge reset_timestamp) begin
    if (final_timer_timestamp != 0 & final_timer_timestamp > timestamp) begin
      // recalculate stop time if timer is running
      final_timer_timestamp = (final_timer_timestamp - timestamp);
    end
  end  

endmodule
