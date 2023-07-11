module tb_realclock();
  // this is a testebench for a timer module named realclock.
  // you may want to change the following variables:
  // - clk_frequency;
  // - timestamp_unit;
  // - finish_clock_test_at_time; and
  // - and test_tolerance.

  logic clk = 0; // initiate in low state.
  parameter real clk_frequency = 1e6; // in Hertz.
  parameter real clk_period = (1e12/clk_frequency); // 1e12 correspound to 1 sec in real datatype scale.

  real timestamp; // fracional timestemp type.
  realtime start_clock_sim_time; // store simulation time to compare with module and check results.
  realtime stop_clock_sim_time;
  
  logic clock_test_completed = 0; // testbench flags.
  logic clock_test_passed = 0;
  real time_dif; // just check results.

  logic reset_timestamp = 0; // controllers for clock module.
  logic enable_clock = 0;

  parameter real timestamp_unit = 1; // make 1 unit of timestamp correspond to this value in seconds.
  parameter real finish_clock_test_at_time = 0.001; // end of test.
  parameter real test_tolerance = 1e-9; // for check results.
  // Information: test initiation requires half a clk_period delay,
  // which is considered during testing and result checking.

  // tested module
  realclock realclock (.clk(clk),
                       .clk_frequency(clk_frequency),
                       .timestamp_unit(timestamp_unit),
                       .reset(reset_timestamp),
                       .enable(enable_clock),
                       .timestamp(timestamp));

  task init_clock_test();
    // run once.
    reset_timestamp = 1; // make a pulse in reset to set internal module states
    #(clk_period/2) enable_clock = 1; // wait half of clk_period to prevent instant increment in timestamp
    reset_timestamp = 0;

    start_clock_sim_time = $realtime; // store to use in checking.
    $display("Clock test started! ", "start_time simulation time: ", start_clock_sim_time, " ps.");
  endtask

  task check_clock();
    // run loop.
    if (timestamp >= (finish_clock_test_at_time / timestamp_unit) & ~clock_test_completed) begin
      // run once, unless clock_test_completed flag is set to high.
      enable_clock = 0;
      clock_test_completed = 1;
      stop_clock_sim_time = $realtime; 
      $display("Clock test completed! ", "stop_time simulation time: ", stop_clock_sim_time, " ps.");
      $display("Clock module timestamp: ", timestamp);

      // this section compares the running time of the simulation, 
      // with the timestamp provided by the module, and test the required tolerance.
      // improvements can be worked on here.
      time_dif = timestamp - ((stop_clock_sim_time - start_clock_sim_time) * timestamp_unit);
      if (time_dif < 0) 
        time_dif = - time_dif;

      if (test_tolerance < time_dif) begin
        clock_test_passed = 1; // testbench flag.
        $display("Clock module PASSED! difference between 'elapsed simulation test time' and 'realclock timestamp' 'less than tolerance'.");
      end else begin
        $display("Clock module REPROVED! difference between 'elapsed simulation test time' and 'realclock timestamp' 'exceeds tolerance'.");
      end 
      $stop;
    end
  endtask

  // hadware initialization
  initial begin
    $display("parameters -> [", 
             "finish_clock_test_at_time: ", finish_clock_test_at_time, " s, ", 
             "clk_frequency: ", clk_frequency, " hz, ",
             "clk_period: ", clk_period, " ps, ",
             "test_tolerance: ", test_tolerance, " s, ",
             "timestamp_unit: ", timestamp_unit, " s",
             "]");
    init_clock_test();
  end

  // generate clk.
  always begin
    #(clk_period/2) clk = ~clk;
  end

  // main loop.
  always @ (posedge clk) begin
    check_clock();
  end                  
endmodule
