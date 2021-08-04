module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    

//==== Parameter ==========================================
    localparam IN_CACHE = 2'b00;
    localparam UPDATE_MEM = 2'b01;
    localparam READ_MEM = 2'b10;
    localparam WRITE_CACHE = 2'b11;

//==== wire/reg definition ================================
    wire [24:0] data_tag;
    wire [2:0] index;
    wire [1:0] block_offset;
    wire hit;
    reg proc_stall;
    reg mem_read, nxt_mem_read;
    reg mem_write, nxt_mem_write;
    reg [27:0] mem_addr, nxt_mem_addr;
    reg [1:0] state, nxt_state;
    reg [31:0] data [0:7][0:3];
    reg [31:0] nxt_data [0:7][0:3];
    reg [25:0] tag [0:7];
    reg [25:0] nxt_tag [0:7];
    reg valid [0:7];
    reg nxt_valid [0:7];
    reg dirty [0:7];
    reg nxt_dirty [0:7];

    integer i, j;
    
//==== combinational circuit ==============================
    assign data_tag = proc_addr[29:5];
    assign index = proc_addr[4:2];
    assign block_offset = proc_addr[1:0];
    assign hit = valid[index] & (tag[index] == data_tag);
    assign proc_rdata = data[index][block_offset];
    assign mem_wdata = {data[index][3], data[index][2], data[index][1], data[index][0]};

    always@(*) begin
        for (i = 0; i < 8; i = i + 1) nxt_valid[i] = valid[i];
        for (i = 0; i < 8; i = i + 1) nxt_tag[i] = tag[i];
        for (i = 0; i < 8; i = i + 1) nxt_dirty[i] = dirty[i];
        for (i = 0; i < 8; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[i][j] = data[i][j];
            end
        end
        case(state)
            IN_CACHE: begin
                if (hit) begin
                    nxt_state = IN_CACHE;
                    proc_stall = 0;
                    nxt_mem_read = 0;
                    nxt_mem_write = 0;
                    nxt_mem_addr = proc_addr[29:2];
                    if (proc_write) begin
                        nxt_dirty[index] = 1;
                        nxt_data[index][block_offset] = proc_wdata;
                    end
                end
                else begin
                    proc_stall = 1;
                    if (dirty[index]) begin
                        nxt_state = UPDATE_MEM;
                        nxt_mem_read = 0;
                        nxt_mem_write = 1;
                        nxt_mem_addr = {tag[index], index};
                    end
                    else begin
                        nxt_state = READ_MEM;
                        nxt_mem_read = 1;
                        nxt_mem_write = 0;
                        nxt_mem_addr = proc_addr[29:2];
                    end
                end
            end
            UPDATE_MEM: begin
                proc_stall = 1;
                if (mem_ready) begin
                    nxt_state = READ_MEM;
                    nxt_dirty[index] = 0;
                    nxt_mem_read = 1;
                    nxt_mem_write = 0;
                    nxt_mem_addr = proc_addr[29:2];
                end
                else begin
                    nxt_state = UPDATE_MEM;
                    nxt_mem_read = 0;
                    nxt_mem_write = 1;
                    nxt_mem_addr = mem_addr;
                end
            end
            READ_MEM: begin
                nxt_mem_addr = proc_addr[29:2];
                if (mem_ready) begin
                    proc_stall = 1;
                    nxt_mem_read = 0;
                    nxt_mem_write = 0;
                    nxt_valid[index] = 1;
                    nxt_tag[index] = data_tag;
                    nxt_data[index][0] = mem_rdata[31:0];
                    nxt_data[index][1] = mem_rdata[63:32];
                    nxt_data[index][2] = mem_rdata[95:64];
                    nxt_data[index][3] = mem_rdata[127:96];
                    if (proc_write) begin
                        nxt_state = WRITE_CACHE;
                    end
                    else begin
                        nxt_state = IN_CACHE;
                    end
                end
                else begin
                    nxt_state = READ_MEM;
                    proc_stall = 1;
                    nxt_mem_read = 1;
                    nxt_mem_write = 0;
                end
            end
            WRITE_CACHE: begin
                nxt_state = IN_CACHE;
                proc_stall = 1;
                nxt_mem_read = 0;
                nxt_mem_write = 0;
                nxt_mem_addr = proc_addr[29:2];
                nxt_dirty[index] = 1;
                nxt_data[index][block_offset] = proc_wdata;
            end
        endcase
    end


//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            state <= IN_CACHE;
            mem_read <= 0;
            mem_write <= 0;
            mem_addr <= 0;
            for (i = 0; i < 8; i = i + 1) valid[i] <= 0;
            for (i = 0; i < 8; i = i + 1) tag[i] <= 0;
            for (i = 0; i < 8; i = i + 1) dirty[i] <= 0;
            for (i = 0; i < 8; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[i][j] <= 0;
                end
            end
        end
        else begin
            state <= nxt_state;   
            mem_read <= nxt_mem_read;
            mem_write <= nxt_mem_write;
            mem_addr <= nxt_mem_addr;
            for (i = 0; i < 8; i = i + 1) valid[i] <= nxt_valid[i];
            for (i = 0; i < 8; i = i + 1) tag[i] <= nxt_tag[i];
            for (i = 0; i < 8; i = i + 1) dirty[i] <= nxt_dirty[i];
            for (i = 0; i < 8; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[i][j] <= nxt_data[i][j];
                end
            end
        end
    end

endmodule
