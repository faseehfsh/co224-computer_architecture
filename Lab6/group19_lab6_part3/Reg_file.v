/*
CO225 Lab 06 (part 2)
Authors : E/19/105 FAHMAN M.H.M.
          E/19/106 FASEEH M.F.M.
*/
`timescale 1ns/100ps

//Reg_File module
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET,BUSYWAIT);
    
    //Port declarations
    input wire [7:0] IN;        //8 bit input
    input [2:0] INADDRESS, OUT1ADDRESS, OUT2ADDRESS;    //3 bit address inputs
    input WRITE, CLK, RESET, BUSYWAIT;    //1 bit inputs
    output [7:0] OUT1, OUT2;    //8 bit outputs
    
    //register array
    reg [7:0] register [7:0]; 

    integer i;  //declared for looping   

    always @(posedge CLK)   //When rising edge of clock
    begin 
        if (WRITE && !BUSYWAIT) begin   //When WRITE is in high
            //Write IN to the given memory address
            case (INADDRESS)
                3'b000 :  register[0] <= #1 IN;
                3'b001 :  register[1] <= #1 IN;
                3'b010 :  register[2] <= #1 IN;
                3'b011 :  register[3] <= #1 IN;
                3'b100 :  register[4] <= #1 IN;
                3'b101 :  register[5] <= #1 IN;
                3'b110 :  register[6] <= #1 IN;
                3'b111 :  register[7] <= #1 IN;
            endcase

            /*
            //Alternate
            #1 register[INADDRESS] = IN;
            */
        end

        if (RESET == 1) begin   //When RESET is in high
            
            #1  for (i=0 ; i<8 ; i++)
                begin
                    register[i] = 8'b00000000;  //WRITE all memory zero
                end
        end
    end

    //READ two parallel data outputs
    assign #2 OUT1 = register[OUT1ADDRESS];
    assign #2 OUT2 = register[OUT2ADDRESS];
endmodule