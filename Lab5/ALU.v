/*
CO225 Lab 05 (part 5)
Authors : E/19/105 FAHMAN M.H.M.
          E/19/106 FASEEH M.F.M.
*/

//ALU module
module alu(DATA1, DATA2, RESULT, SELECT, ZERO, SHIFTINDICATOR);
    //Port declarations
    input [7:0] DATA1, DATA2;   //8 bit inputs
    input [2:0] SELECT;         //3 bit input
    input SHIFTINDICATOR;       //1 bit input (indicator for sll & srl)
    output reg ZERO;                //1 bit output (indicator for beq) 
    output reg [7:0] RESULT;    //8 bit output
    wire [7:0] Fwd,Add,And,Or,LogicalShift,ArithmeticShift,Rotate;  //8 bit temporary variables
    output reg zero_flag;       //to check invalid OP code

    /*
    perform all operations and get results of each
    operation by instantiating lower level modules
    */
    FORWARD Forward_(DATA2, Fwd);
    ADD Add_(DATA1, DATA2, Add);
    AND And_(DATA1, DATA2, And);
    OR Or_(DATA1, DATA2, Or);
    LOGICALSHIFT LogicalShift_(DATA1, DATA2, SHIFTINDICATOR, LogicalShift);
    ARITHMETICSHIFT ArithmeticShift_(DATA1, DATA2, ArithmeticShift);
    ROTATE Rotate_(DATA1, DATA2, Rotate);

    /*Check whether the result of substraction operation is zero or not,
      in order to implement the beq instruction
    */
    always @(*) begin
        if (Add==0) begin
            ZERO=1;
        end else begin
            ZERO=0;
        end
        
    end
    

    /*
    According to the OP code 
    switch the appropriate value to the result
    using MUX
    */
    always @(*)
    begin
        zero_flag = 1'b0;       //initialize to 0

        case (SELECT)
            3'b000:  RESULT = Fwd;
            3'b001:  RESULT = Add;
            3'b010:  RESULT = And;
            3'b011:  RESULT = Or;
            3'b100:  RESULT = LogicalShift;
            3'b101:  RESULT = ArithmeticShift;
            3'b110:  RESULT = Rotate;
            default: zero_flag = 1'b1;  //value to 1
        endcase 
        /*Check for invalid P code.
        if invalid OP code is detected, terminate the program
        */
        if (zero_flag == 1) begin
            $display("Invalid OP code..");
            $finish;
        end
    end
endmodule
/*
----------Lower level modules-----------
*/
//FORWARD module
module FORWARD(DATA2, RESULT);
    //Port declarations
    input [7:0] DATA2;
    output reg [7:0] RESULT;
    
    always @(DATA2)
    begin
        #1 RESULT = DATA2;
    end 
endmodule

//ADD module
module ADD(DATA1, DATA2, RESULT);
    //Port declarations
    input [7:0] DATA1, DATA2;
    output reg [7:0] RESULT;

    always @(DATA1, DATA2)
    begin
        #2 RESULT = DATA1 + DATA2;
    end
endmodule

//AND module
module AND(DATA1, DATA2, RESULT);
   //Port declarations
    input [7:0] DATA1, DATA2;
    output reg [7:0] RESULT;

    always @(DATA1, DATA2)
    begin
        #1 RESULT = DATA1 & DATA2;
    end
endmodule

//OR module
module OR(DATA1, DATA2, RESULT);
   //Port declarations
    input [7:0] DATA1, DATA2;
    output reg [7:0] RESULT;

    always @(DATA1, DATA2)
    begin
        #1 RESULT = DATA1 | DATA2;
    end
endmodule

//Logical Shift module
module LOGICALSHIFT (OPERAND, SHIFTAMOUNT, SHIFTSIDE, RESULT);
    //Port declarations
    input [7:0] OPERAND, SHIFTAMOUNT;
    input SHIFTSIDE;                    //If 0 leftshift, if 1 rightshift
    output reg [7:0] RESULT;
    reg [7:0] tempOperand;              //temporary register
    reg [6:0] temp;                     //temporary register
    integer i;                          //for looping

    always @(SHIFTAMOUNT, OPERAND) begin
        #1
        tempOperand = OPERAND;
        for (i = 1; i<=SHIFTAMOUNT; i++ ) begin
            if (SHIFTSIDE == 0) begin       //Leftshift operation
                temp = tempOperand[6:0];
                RESULT[7:1] = temp;
                RESULT[0] = 1'b0;
                tempOperand = RESULT;
            end else begin                  //Rightshift operation
                temp = tempOperand[7:1];
                RESULT[6:0] = temp;
                RESULT[7] = 1'b0;
                tempOperand = RESULT;
            end
        end
    end
endmodule

//Arithmetic Shift module
module ARITHMETICSHIFT(OPERAND, SHIFTAMOUNT, RESULT);
    //Port declarations
    input [7:0] OPERAND, SHIFTAMOUNT;
    output reg [7:0] RESULT;
    reg [7:0] tempOperand;          //temporary register 
    reg [6:0] temp;                 //temporary register
    integer j;                      //for looping

    always @(OPERAND, SHIFTAMOUNT) begin
        #1
        tempOperand = OPERAND;
        for (j = 1; j<=SHIFTAMOUNT ; j++) begin
            if (tempOperand[7]==1'b0) begin     //If MSB is 0 add leading zeros
                temp = tempOperand[7:1];
                RESULT[6:0] = temp;
                RESULT[7] = 1'b0;
                tempOperand = RESULT;
            end else begin                      //If MSB is 1 add leading ones
                temp = tempOperand[7:1];
                RESULT[6:0] = temp;
                RESULT[7] = 1'b1;
                tempOperand = RESULT;
            end
        end
    end
endmodule

//Rotate right module
module ROTATE(OPERAND, ROTATEAMOUNT, RESULT);
    input [7:0] OPERAND, ROTATEAMOUNT;
    output reg [7:0] RESULT;
    reg tempLSB;                    //temporary register
    reg [6:0] temp;                 //temporary register
    reg [7:0] tempOperand;          //temporary register
    integer k;                      //for looping

    always @(OPERAND, ROTATEAMOUNT) begin
        #1
        tempOperand = OPERAND;      
        tempLSB = OPERAND[0];       //Save the LSB 

        //ROtate through a loop
        for ( k=1 ; k<=ROTATEAMOUNT ; k++) begin 
            temp = tempOperand[7:1];        
            RESULT[6:0] = temp;
            RESULT[7] = tempLSB;
            tempOperand = RESULT;
            tempLSB = RESULT[0];
        end
    end
endmodule