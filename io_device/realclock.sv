module realclock(input logic clk,
                 input real clk_frequency,
                 input real timestamp_unit,
                 input logic reset,
                 input logic enable,
                 output real timestamp);

  // convert clks to fractional timestemp
  always @ (negedge clk) begin
    if (enable) begin
      timestamp <= timestamp + (1 / (timestamp_unit*clk_frequency));
    end
  end

  // reset runs like a pulse command
  always @ (posedge reset) begin
      timestamp = 0;
  end
endmodule
