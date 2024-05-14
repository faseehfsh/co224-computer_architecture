/*
CO225 Lab 05 (part 5)
Authors : E/19/105 FAHMAN M.H.M.
          E/19/106 FASEEH M.F.M.
*/
`include "ALU.V"            //Import ALU module
`include "Reg_file.v"       //Import Reg_file module


//Testbench for cpu module
module cpu_tb;
    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    

    reg [7:0] instr_mem[0:1023];
  
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)

    always @(PC) begin
        #2
        INSTRUCTION = {instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};
    end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // manually loading instructions to instr_mem
        
        {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]}        = 32'b00000000_00000000_00000000_00001001;  //loadi 0     #9
        {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]}        = 32'b00000110_00000001_00000000_00000000;  //j     #1     
        {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]}      = 32'b00000000_00000001_00000000_00000100;  //loadi 1     #4
        {instr_mem[10'd15], instr_mem[10'd14], instr_mem[10'd13], instr_mem[10'd12]}    = 32'b00000000_00000001_00000000_00001010;  //loadi 1     #10  
        {instr_mem[10'd19], instr_mem[10'd18], instr_mem[10'd17], instr_mem[10'd16]}    = 32'b00000111_00000001_00000001_00000000;  //beq   #1  1  0
        {instr_mem[10'd23], instr_mem[10'd22], instr_mem[10'd21], instr_mem[10'd20]}    = 32'b00000000_00000010_00000000_00001001;  //loadi 2     #9
        {instr_mem[10'd27], instr_mem[10'd26], instr_mem[10'd25], instr_mem[10'd24]}    = 32'b00000111_00000001_00000010_00000000;  //beq   #1  2  0
        {instr_mem[10'd31], instr_mem[10'd30], instr_mem[10'd29], instr_mem[10'd28]}    = 32'b00000011_00000011_00000000_00000001;  //sub   3   0  1
        {instr_mem[10'd35], instr_mem[10'd34], instr_mem[10'd33], instr_mem[10'd32]}    = 32'b00000010_00000011_00000000_00000001;  //add   3   0  1
        {instr_mem[10'd39], instr_mem[10'd38], instr_mem[10'd37], instr_mem[10'd36]}    = 32'b00001001_00000100_00000001_00000010;  //srl   4   1  #2   
        {instr_mem[10'd43], instr_mem[10'd42], instr_mem[10'd41], instr_mem[10'd40]}    = 32'b00001010_00000101_00000010_00000010;  //sra   5   2  #2   
        {instr_mem[10'd47], instr_mem[10'd46], instr_mem[10'd45], instr_mem[10'd44]}    = 32'b00001011_00000001_00000100_00000011;  //bne   #1  4  3
        {instr_mem[10'd51], instr_mem[10'd50], instr_mem[10'd49], instr_mem[10'd48]}    = 32'b00000000_00000110_00000000_00000011;  //loadi 6      #3
        {instr_mem[10'd55], instr_mem[10'd54], instr_mem[10'd53], instr_mem[10'd52]}    = 32'b00000000_00000111_00000000_00001000;  //loadi 7      #8
        {instr_mem[10'd59], instr_mem[10'd58], instr_mem[10'd57], instr_mem[10'd56]}    = 32'b00001100_00000111_00000010_00000011;  //ror   7   2  #3 
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb, cpu_tb.mycpu.REGISTER.register[0], cpu_tb.mycpu.REGISTER.register[1], cpu_tb.mycpu.REGISTER.register[2], cpu_tb.mycpu.REGISTER.register[3], cpu_tb.mycpu.REGISTER.register[4], cpu_tb.mycpu.REGISTER.register[5], cpu_tb.mycpu.REGISTER.register[6], cpu_tb.mycpu.REGISTER.register[7]);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        #2
        RESET = 1'b1;

        #4
        RESET= 1'b0;
        
        // finish simulation after some time
        #500
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
endmodule


//cpu module
module cpu(PC, INSTRUCTION, CLK, RESET);
    //Port&wire declarations
    input [31:0] INSTRUCTION;
    input RESET, CLK;
    output reg [31:0] PC;
    reg [2:0] ALU_SELECT;
    wire [7:0] OPERAND1,OPERAND2,RESULT,MUX1OUT,MUX2OUT;
    reg [7:0] twosComplement ,IMMEDIATE, JUMPCOUNT, OPCODE;
    reg [2:0] SOURCE1ADDRESS,SOURCE2ADDRESS,DESTINATIONADDRESS;
    reg WRITE, isSUBSTRACT, isIMMEDIATE, isJUMP, isBEQ, isBNE, isRIGHTSHIFT;
    wire [31:0] PCPLUSFOUR, NEXTPC, PCJUMP, EXTENDEDJUMPCOUNT, FINALJUMPCOUNT;
    wire ZERO;

    //get the increased PC value (+4)
    next_PC NEXTPCVAl(PC, PCPLUSFOUR);
    //directs the operand2 value if addition else directs the 2's complement value if substraction
    mux MUX1(twosComplement, OPERAND2, isSUBSTRACT, MUX1OUT);
    //directs the immediate value if operand2 is an immediate value else directs the MUX1OUT
    mux MUX2(IMMEDIATE, MUX1OUT, isIMMEDIATE, MUX2OUT);
    //Assign PC to the address of subsequent instruction or jump to another instruction
    mux_pc MUX3(PCJUMP, PCPLUSFOUR, isJUMP, isBEQ, isBNE, ZERO, NEXTPC);
    //Register file to store and retrieve data and results
    reg_file REGISTER(RESULT,OPERAND1,OPERAND2,DESTINATIONADDRESS,SOURCE1ADDRESS,SOURCE2ADDRESS,WRITE,CLK,RESET);
    //do ALU operations according to the ALU_SELECT value derived from OPCODE
    alu ALU(OPERAND1, MUX2OUT, RESULT, ALU_SELECT, ZERO, isRIGHTSHIFT);
    //Extend the offset to 32 bits
    extend EXTENDOFFSET(JUMPCOUNT, EXTENDEDJUMPCOUNT);
    //getting the offset value by multiplying the no of instructions to be jumped by 4
    shift SHIFT(EXTENDEDJUMPCOUNT, FINALJUMPCOUNT);
    //get the target address for j, beq instructions
    jump JUMPTO(PCPLUSFOUR, PCJUMP, FINALJUMPCOUNT);       


    //If reset=1 set the PC value to 0 synchronously with a delay of 1
    always @(posedge CLK) begin
        #1
        if(RESET) PC <= 32'd0; 
    end
    //Increase the PC value by for and assign it to the same PC synchronously with a delay of 1
    always @(posedge CLK) begin
        #1
        PC <= NEXTPC;
    end
    //Results the 2's complement value of OPERAND2
    always @(OPERAND2) begin
        #1 twosComplement = ~(OPERAND2)+1;        
    end

    //Instruction fetching mechanism
    always @(INSTRUCTION) begin
        OPCODE = INSTRUCTION[31:24];
        DESTINATIONADDRESS = INSTRUCTION[18:16];
        SOURCE1ADDRESS = INSTRUCTION[10:8];
        SOURCE2ADDRESS = INSTRUCTION[2:0];
        IMMEDIATE = INSTRUCTION[7:0];
        JUMPCOUNT = INSTRUCTION[23:16];

        #1
        case(OPCODE)
            // LOAD
            8'd0 : 
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b000;
                    WRITE = 1;
                end
            // MOV
            8'd1 : 
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b000;
                    WRITE = 1;
                end
            // ADD
            8'd2 : 
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b001;
                    WRITE = 1;
                end
            // SUB
            8'd3 : 
                begin
                    isSUBSTRACT = 1'b1;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b001;
                    WRITE = 1;
                end
            // AND
            8'd4 : 
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b010;
                    WRITE = 1;
                end
            // OR
            8'd5 : 
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b011;
                    WRITE = 1;
                end
            //j
            8'd6 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b1;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    WRITE = 0;
                end
            //beq
            8'd7 :
                begin
                    isSUBSTRACT = 1'b1;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b1;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b001;
                    WRITE = 0;
                end
            //sll
            8'd8 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b100;
                    isRIGHTSHIFT = 1'b0;
                    WRITE = 1;
                    
                end
            //srl
            8'd9 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b100;
                    isRIGHTSHIFT = 1'b1;
                    WRITE = 1;
                    
                end
            //sra
            8'd10 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b101;
                    WRITE = 1;
                end
            //bne
            8'd11 :
                begin
                    isSUBSTRACT = 1'b1;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b1;
                    ALU_SELECT = 3'b001;
                    WRITE = 0;
                end
            //ror
            8'd12 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b110;
                    WRITE = 1;
                end
        endcase
    end
endmodule

//MUX module
module mux(DATA1,DATA2,SELECT,OUTPUT);
    //Declaration of ports
    input [7:0] DATA1,DATA2;
    input SELECT;
    output reg [7:0] OUTPUT;

    //direct on of the data to the output according to the select value
    always@(*) begin
        case(SELECT)
            1'b1: OUTPUT <= DATA1;
            1'b0: OUTPUT <= DATA2;
            default: OUTPUT <= 8'd0;
        endcase
    end
endmodule

//MUX module for jump and branch instructions
module mux_pc(DATA1,DATA2,JUMPSELECT,BEQSELECT,BNESELECT,ZEROSELECT,OUTPUT);
    //Declaration of ports
    input [31:0] DATA1,DATA2;
    input JUMPSELECT, BEQSELECT, BNESELECT, ZEROSELECT;
    output reg [31:0] OUTPUT;

    //direct on of the data to the output according to the select value
    always@(*) begin
        if (JUMPSELECT) begin
            OUTPUT = DATA1;                         //If the instruction is j, assign the NextPC value to the target address
        end else if (BEQSELECT && ZEROSELECT) begin //If the instruction is beq and equal condition is satisfied,
            OUTPUT = DATA1;                         // assign the NextPC value to the target address
        end else if (BNESELECT && !ZEROSELECT) begin    //If the instruction is bne and notequal condition is satisfied
            OUTPUT = DATA1;                             // assign the NextPC value to the target address
        end else begin                              
            OUTPUT = DATA2;                         //Else nextPC value is increased by 4 from currentPC value
        end
        
    end
endmodule

//Next PC module
module next_PC(IN,OUT);
    input [31:0] IN;
    output reg [31:0] OUT;

    always@(IN) begin
        #1 OUT <= IN + 32'b100;  //increases the PC value by 4
    end
endmodule

//JUMP module for j , beq instructions
module jump(CURRENTPC, TARGETPC, JUMPBY);
    input [31:0] CURRENTPC, JUMPBY;
    output wire [31:0] TARGETPC;

    assign #2 TARGETPC = CURRENTPC + JUMPBY;   //Calculate the target address
endmodule

//Sign extend the offset value from 8 bits to 32 bits
module extend(OFFSET, EXTENDEDOFFSET);
    input [7:0] OFFSET;
    output reg [31:0] EXTENDEDOFFSET;

    always @(OFFSET) begin
        EXTENDEDOFFSET [7:0] = OFFSET;
        if (OFFSET[7]) begin
            EXTENDEDOFFSET [31:8] = 24'b111111111111111111111111;   //append 1's if MSB is 1
        end else begin
            EXTENDEDOFFSET [31:8] = 24'b000000000000000000000000;   //append 0's if MSB is 0
        end
    end
endmodule

//Shift module to shift the input number left by 2
module shift(VALUE, SHIFTEDVALUE);
    input signed [31:0] VALUE;
    output signed [31:0] SHIFTEDVALUE;

    assign SHIFTEDVALUE = VALUE<<2; //Left shift by 2 bits (mutiply by 4)
endmodule
