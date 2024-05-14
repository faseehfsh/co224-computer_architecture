/*
CO225 Lab 06 (part 2)
Authors : E/19/105 FAHMAN M.H.M.
          E/19/106 FASEEH M.F.M.
*/
`include "ALU.V"            //Import ALU module
`include "Reg_file.v"       //Import Reg_file module
`include "Data_mem.v"       //Import Data_mem module
`include "dcache.v"         //import dcache module
`include "icache.v"         //Import icache module
`include "Instruction_mem.v"//Import Instruction_mem module
`timescale 1ns/100ps

//Testbench for cpu module
module cpu_tb;
    reg CLK, RESET;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;

    wire [5:0] MEM_ADDRESS;
    wire [31:0] MEM_WRITEDATA, MEM_READDATA;
    wire MEM_WRITE, MEM_READ, MEM_BUSYWAIT;

    wire [7:0] ADDRESS;
    wire [7:0] CPU_WRITEDATA, CPU_READDATA;
    wire READ, WRITE, BUSYWAIT;

    wire INSTR_BUSYWAIT;
	
	wire IM_READ, IM_BUSYWAIT;
	wire [5:0] IM_ADDRESS;
	wire [127:0] IM_INSTR;
    
    
   
    /* 
    -----
     Data cache MEMORY
    -----
    */
    dcache mydatacache(CPU_WRITEDATA, CPU_READDATA, ADDRESS, READ, WRITE, BUSYWAIT, MEM_WRITEDATA, MEM_READDATA, MEM_ADDRESS, MEM_READ, MEM_WRITE, MEM_BUSYWAIT, CLK, RESET);


    /* 
    -----
     DATA MEMORY
    -----
    */
    data_memory mydatamemory(CLK, RESET, MEM_READ, MEM_WRITE, MEM_ADDRESS, MEM_WRITEDATA, MEM_READDATA, MEM_BUSYWAIT);


    /*
	------------------
	INSTRUCTION CACHE
	------------------
    */
	icache my_icache(CLK, RESET, PC[9:0], INSTRUCTION, INSTR_BUSYWAIT, IM_READ, IM_ADDRESS, IM_INSTR, IM_BUSYWAIT);
	

    /*
	------------------
	INSTRUCTION MEMORY
	------------------
    */
	instruction_memory my_imemory(CLK, IM_READ, IM_ADDRESS, IM_INSTR, IM_BUSYWAIT);


    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET, CPU_WRITEDATA, CPU_READDATA, ADDRESS, WRITE, READ, BUSYWAIT, INSTR_BUSYWAIT);
    
    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb, cpu_tb.mycpu.REGISTER.register[0], cpu_tb.mycpu.REGISTER.register[1], cpu_tb.mycpu.REGISTER.register[2], cpu_tb.mycpu.REGISTER.register[3], cpu_tb.mycpu.REGISTER.register[4], cpu_tb.mycpu.REGISTER.register[5], cpu_tb.mycpu.REGISTER.register[6], cpu_tb.mycpu.REGISTER.register[7],
        cpu_tb.mydatacache.cacheblock_array[0], cpu_tb.mydatacache.cacheblock_array[1], cpu_tb.mydatacache.cacheblock_array[2], cpu_tb.mydatacache.cacheblock_array[3], cpu_tb.mydatacache.cacheblock_array[4], cpu_tb.mydatacache.cacheblock_array[5], cpu_tb.mydatacache.cacheblock_array[6], cpu_tb.mydatacache.cacheblock_array[7],
        cpu_tb.mydatamemory.memory_array[0], cpu_tb.mydatamemory.memory_array[1], cpu_tb.mydatamemory.memory_array[2], cpu_tb.mydatamemory.memory_array[3], cpu_tb.mydatamemory.memory_array[4], cpu_tb.mydatamemory.memory_array[5], cpu_tb.mydatamemory.memory_array[6], cpu_tb.mydatamemory.memory_array[7], cpu_tb.mydatamemory.memory_array[8],
        cpu_tb.mydatacache.dirty_array[0], cpu_tb.mydatacache.dirty_array[1], cpu_tb.mydatacache.dirty_array[2], cpu_tb.mydatacache.dirty_array[3], cpu_tb.mydatacache.dirty_array[4], cpu_tb.mydatacache.dirty_array[5], cpu_tb.mydatacache.dirty_array[6], cpu_tb.mydatacache.dirty_array[7], 
        cpu_tb.mydatacache.valid_array[0], cpu_tb.mydatacache.valid_array[1], cpu_tb.mydatacache.valid_array[2], cpu_tb.mydatacache.valid_array[3], cpu_tb.mydatacache.valid_array[4], cpu_tb.mydatacache.valid_array[5], cpu_tb.mydatacache.valid_array[6], cpu_tb.mydatacache.valid_array[7], 
        cpu_tb.mydatacache.tagArray[0], cpu_tb.mydatacache.tagArray[1], cpu_tb.mydatacache.tagArray[2], cpu_tb.mydatacache.tagArray[3], cpu_tb.mydatacache.tagArray[4], cpu_tb.mydatacache.tagArray[5], cpu_tb.mydatacache.tagArray[6], cpu_tb.mydatacache.tagArray[7] );
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        #2
        RESET = 1'b1;

        #4
        RESET= 1'b0;
        
        // finish simulation after some time
        #5000
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
endmodule


//cpu module
module cpu(PC, INSTRUCTION, CLK, RESET, WRITEDATA, READDATA, ADDRESS, WRITE_D, READ_D, BUSYWAIT, INSTR_BUSYWAIT);
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
    //Extra ports & wires created for Data memory access (Lab6)
    output [7:0] WRITEDATA;
    output [7:0] ADDRESS;
    input [7:0] READDATA;
    output reg WRITE_D, READ_D;
    input BUSYWAIT;
    wire [7:0] REGWRITEDATA;
    //Extra ports & wires crreated for instruction memory access (Lab6)
    input INSTR_BUSYWAIT;

        
    
    //get the increased PC value (+4)
    next_PC NEXTPCVAl(PC, PCPLUSFOUR);
    //directs the operand2 value if addition else directs the 2's complement value if substraction
    mux MUX1(twosComplement, OPERAND2, isSUBSTRACT, MUX1OUT);
    //directs the immediate value if operand2 is an immediate value else directs the MUX1OUT
    mux MUX2(IMMEDIATE, MUX1OUT, isIMMEDIATE, MUX2OUT);
    //Assign PC to the address of subsequent instruction or jump to another instruction
    mux_pc MUX3(PCJUMP, PCPLUSFOUR, isJUMP, isBEQ, isBNE, ZERO, NEXTPC);
    //Select the register write data from data memory or alu output 
    mux MUX4(READDATA, RESULT, READ_D, REGWRITEDATA);
    //Register file to store and retrieve data and results
    reg_file REGISTER(REGWRITEDATA,OPERAND1,OPERAND2,DESTINATIONADDRESS,SOURCE1ADDRESS,SOURCE2ADDRESS,WRITE,CLK,RESET,BUSYWAIT);
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
        if(RESET) #1 PC <= 32'd0; 
    end
    //Increase the PC value by for and assign it to the same PC synchronously with a delay of 1
    always @(posedge CLK) begin
        if(~(BUSYWAIT|INSTR_BUSYWAIT)) begin
            #1 PC <= NEXTPC;
        end
    end
    //Results the 2's complement value of OPERAND2
    always @(OPERAND2) begin
        #1 twosComplement = ~(OPERAND2)+1;        
    end
    //When data memory is not busy set the READ & WRITE signals to zero
    always @(BUSYWAIT) begin
        if (~BUSYWAIT) begin
            READ_D = 1'b0;
            WRITE_D = 1'b0;
        end
    end


    assign WRITEDATA = OPERAND1;        
    assign ADDRESS = RESULT;
    

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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;                    
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
                end
            //j
            8'd6 :
                begin
                    isJUMP = 1'b1;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;   
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0; 
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
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
                    READ_D = 1'b0;
                    WRITE_D = 1'b0;
                end
            //lwd
            8'd13 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b000;
                    WRITE = 1'b1;
                    READ_D = 1'b1;
                    WRITE_D = 1'b0;    
                end
            //lwi
            8'd14 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b000;
                    WRITE = 1'b1;
                    READ_D = 1'b1;
                    WRITE_D = 1'b0;    
                end
            //swd
            8'd15 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b000;
                    WRITE = 1'b0;
                    READ_D = 1'b0;
                    WRITE_D = 1'b1;    
                end
            //swi
            8'd16 :
                begin
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isJUMP = 1'b0;
                    isBEQ = 1'b0;
                    isBNE = 1'b0;
                    ALU_SELECT = 3'b000;
                    WRITE = 1'b0;
                    READ_D = 1'b0;
                    WRITE_D = 1'b1;    
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

    //direct one of the data to the output according to the select value
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
