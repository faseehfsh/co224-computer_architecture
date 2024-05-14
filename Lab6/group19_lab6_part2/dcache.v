/*
Module  : Data Cache 
Main Author  : Isuru Nawinne, Kisaru Liyanage
Date    : 25/05/2020

CO225 Lab 06 (part 2)
Authors : E/19/105 FAHMAN M.H.M.
          E/19/106 FASEEH M.F.M.

*/
`timescale 1ns/100ps

module dcache(cpu_writeData, cpu_readData, address, read, write, busywait, mem_writedata, mem_readdata, mem_address, mem_read, mem_write, mem_busywait, clock, reset);
    //Port declarations
    //CPU
    input [7:0] cpu_writeData, address;
    input write, read, clock, reset;
    output reg [7:0] cpu_readData;
    output reg busywait;

    //DATA MEMORY
    input [31:0] mem_readdata;
    input mem_busywait;
    output reg [31:0] mem_writedata;
    output reg [5:0] mem_address;
    output reg mem_read, mem_write;

    wire [2:0] tag, index, cache_tag;
    wire [1:0] offset;
    wire dirty, hit;
    reg  tagmatch, cache_write;




    // Cache Data Sturcture
    reg [31:0] cacheblock_array [7:0];  // block array (32bit * 8)
    reg dirty_array [7:0];              // dirty bit array (1*8)
    reg  valid_array [7:0];             // valid bit array (1*8)
    reg [2:0] tagArray [7:0];           // tag array (3*8)

    // split the address into tag , index , offset.
    assign #1 tag = address[7:5];
    assign #1 index = address[4:2];
    assign #1 offset = address[1:0];

    //get the dirty bit form the array
    assign #1 dirty = dirty_array[index];

    // set busywait high when write or read signal is detected
    always @(read,write) begin
        busywait = (read || write)? 1 : 0;
    end

    // do the tag comparision and hit generation with parallel to the cache read
    always @(index,tagArray[index],tag) begin
        #0.9
        if(tag==tagArray[index])begin
            tagmatch = 1 ;
        end
        else begin
            tagmatch = 0 ;
        end
    end
    assign hit = tagmatch & valid_array[index];  // perform the hit generation

    // route relavant data value into cpu_read_data as soon as indexing happend 
    always @(*) begin
        case (offset)
            // according to the offset give the relavant cpuread valu
            0: cpu_readData = cacheblock_array[index][7:0] ;
            1: cpu_readData = cacheblock_array[index][15:8] ;
            2: cpu_readData = cacheblock_array[index][23:16] ;
            3: cpu_readData = cacheblock_array[index][31:24] ;
        endcase
    end  

    always @(*) begin
        if(hit) begin
            if(read && (!write)) begin
                busywait = 0 ;  // set the bussyWait = 0 as soon as hit detected
                tagmatch = 0 ;  // set the tagmatch = 0 as soon as hit detected
                                // unless this reason to regenerate the hit with last value of (tag , index, ..)
            end
            else if (write && (!read)) begin
                busywait = 0;       // set the busywait = 0 as soon as hit detected
                cache_write = 1;    // decide whether write hit or not 
                
            end
        end
    end

    always @(posedge clock) begin
        // Write data on the relavant place of the cache at the next positive edge of the clock
       if(cache_write==1) begin
           #1
            case (offset)
                0: cacheblock_array[index][7:0] = cpu_writeData;
                1: cacheblock_array[index][15:8] = cpu_writeData;
                2: cacheblock_array[index][23:16] = cpu_writeData;
                3: cacheblock_array[index][31:24] = cpu_writeData;
            endcase
           dirty_array[index] = 1;
       end 
    end



    

    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE=3'b010;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((read || write) && !dirty && !hit)  
                    next_state = MEM_READ;
                else if ((read || write) && dirty && !hit)
                    next_state = MEM_WRITE;
                else
                    next_state = IDLE;
            
           MEM_READ:
                if (!mem_busywait)
                    next_state = IDLE;
                else    
                    next_state = MEM_READ;
            MEM_WRITE:
                if (!mem_busywait)
                    next_state = MEM_READ;
                else    
                    next_state = MEM_WRITE;
            
        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 8'dx;
                mem_writedata = 8'dx;
                //busywait = 0;
            end
         
            MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {tag, index};
                mem_writedata = 32'dx;
                busywait = 1;
                
                // waiting untill mem_busywait =0 and then taking #1 unit read data write on the cache.
                // same time valid bit and the tag array bit is changed with relavant value
                #1 if(mem_busywait==0) begin
                    cacheblock_array[index]  = mem_readdata ;
                    valid_array[index] = 1 ;
                    tagArray[index] = tag ;
                end

            end
            MEM_WRITE:
            begin
                mem_read = 0;
                mem_write = 1;
                // pass into data memory with rleated address
                mem_address = {cache_tag, index};
                mem_writedata = cacheblock_array[index];
                busywait = 1;

                // when mem_busywait became zero dirtybit set as zero..
                if(mem_busywait==0) begin
                    dirty_array[index] = 0;
                end

            end
            
        endcase
    end

    // sequential logic for state transitioning
	integer i;
    always @(posedge clock, reset)
    begin
        if(reset) begin
            state = IDLE;
            for(i = 0 ; i<7 ;i = i+1) begin
                dirty_array[i] = 0 ;
                valid_array[i] = 0 ;
            end
        end
        else
            state = next_state;

    end
    /* Cache Controller FSM End */

endmodule