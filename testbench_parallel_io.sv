module testbench();
    logic        clk;
    logic [31:0] WriteData, DataAdr;
    logic        MemWrite;


  // new signals
    logic [1:0]  KEY;
    logic [9:0]  SW;
    logic [9:0]  LEDR;
    logic [31:0] HEX3HEX0;
    logic [15:0] HEX5HEX4;

    top dut(
        .clk(clk), 
        .WriteData(WriteData), 
        .DataAdr(DataAdr), 
        .MemWrite(MemWrite),
        .KEY(KEY),
        .SW(SW),
        .LEDR(LEDR),
        .HEX3HEX0(HEX3HEX0),
        .HEX5HEX4(HEX5HEX4)
    );

    // Initialize test
    initial begin
        // EPWAVE / GTKWave Setup
        $dumpfile("dump.vcd"); 
        $dumpvars(0, dut); 

        // Initialize inputs
      SW[9:0] = 10'b1111111111;
        KEY = 2'b11; 

        KEY[0] = 0;         # 22; 
        KEY[0] = 1; 
    end

    // Generate clock to sequence tests
    always begin
        clk <= 1; # 5; clk <= 0; # 5;
    end


    always @(negedge clk) begin
        if(MemWrite) begin
            if(DataAdr === 100 & WriteData === 25) begin
                $display("Simulation succeeded");
                $stop;
            end
                $display("Simulation failed");
                $display("Attempted write to Adr: %h with Data: %h", DataAdr, WriteData);
                $stop;
            end
        end
    //end
endmodule
