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
    wire [25:0] data_tag;
    wire [1:0] index;
    wire [1:0] block_offset;
    wire hit0, hit1, hit;
    wire set_choose_temp;
    reg proc_stall;
    reg mem_read, nxt_mem_read;
    reg mem_write, nxt_mem_write;
    reg [27:0] mem_addr, nxt_mem_addr;
    reg [1:0] state, nxt_state;
    reg [31:0] data [0:1][0:3][0:3];
    reg [31:0] nxt_data [0:1][0:3][0:3];
    reg [25:0] tag [0:1][0:3];
    reg [25:0] nxt_tag [0:1][0:3];
    reg valid [0:1][0:3];
    reg nxt_valid [0:1][0:3];
    reg dirty [0:1][0:3];
    reg nxt_dirty [0:1][0:3];
    reg set_choose, nxt_set_choose;
    reg set_use [0:3];
    reg nxt_set_use [0:3];
    
    integer i, j;

//==== combinational circuit ==============================
    assign data_tag = proc_addr[29:4];
    assign index = proc_addr[3:2];
    assign block_offset = proc_addr[1:0];
    assign hit0 = valid[0][index] & (tag[0][index] == data_tag);
    assign hit1 = valid[1][index] & (tag[1][index] == data_tag);
    assign hit = hit0 ^ hit1;
    assign proc_rdata = (hit0) ? data[0][index][block_offset] : data[1][index][block_offset];
    assign mem_wdata = {data[set_choose][index][3], data[set_choose][index][2], data[set_choose][index][1], data[set_choose][index][0]};
    //assign set_choose_temp = (~valid[0][index]) ? 0 : ( (~valid[1][index]) ? 1 : ~set_choose );
    assign set_choose_temp = ~set_use[index];

    always@(*) begin
        for (i = 0; i < 4; i = i + 1) nxt_valid[0][i] = valid[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_valid[1][i] = valid[1][i];
        for (i = 0; i < 4; i = i + 1) nxt_tag[0][i] = tag[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_tag[1][i] = tag[1][i];
        for (i = 0; i < 4; i = i + 1) nxt_dirty[0][i] = dirty[0][i];
        for (i = 0; i < 4; i = i + 1) nxt_dirty[1][i] = dirty[1][i];
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[0][i][j] = data[0][i][j];
            end
        end
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                nxt_data[1][i][j] = data[1][i][j];
            end
        end
        nxt_set_choose = set_choose;
        for (i = 0; i < 4; i = i + 1) nxt_set_use[i] = set_use[i];
        case(state)
            IN_CACHE: begin
                if (hit) begin
                    nxt_state = IN_CACHE;
                    proc_stall = 0;
                    nxt_mem_read = 0;
                    nxt_mem_write = 0;
                    nxt_mem_addr = proc_addr[29:2];
                    if (proc_write) begin
                        if (hit0) nxt_dirty[0][index] = 1;
                        else nxt_dirty[1][index] = 1;
                        if (hit0) nxt_data[0][index][block_offset] = proc_wdata;
                        else nxt_data[1][index][block_offset] = proc_wdata;
                    end
                end
                else begin
                    proc_stall = 1;
                    nxt_set_choose = set_choose_temp;
                    if (dirty[set_choose_temp][index]) begin
                        nxt_state = UPDATE_MEM;
                        nxt_mem_read = 0;
                        nxt_mem_write = 1;
                        nxt_mem_addr = {tag[set_choose_temp][index], index};
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
                    nxt_dirty[set_choose][index] = 0;
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
                    nxt_valid[set_choose][index] = 1;
                    nxt_tag[set_choose][index] = data_tag;
                    nxt_data[set_choose][index][0] = mem_rdata[31:0];
                    nxt_data[set_choose][index][1] = mem_rdata[63:32];
                    nxt_data[set_choose][index][2] = mem_rdata[95:64];
                    nxt_data[set_choose][index][3] = mem_rdata[127:96];
                    nxt_set_use[index] = set_choose;
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
                nxt_dirty[set_choose][index] = 1;
                nxt_data[set_choose][index][block_offset] = proc_wdata;
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
            for (i = 0; i < 4; i = i + 1) valid[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) tag[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) dirty[0][i] <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[0][i][j] <= 0;
                end
            end
            for (i = 0; i < 4; i = i + 1) valid[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) tag[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) dirty[1][i] <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[1][i][j] <= 0;
                end
            end
            set_choose <= 0;
            for (i = 0; i < 4; i = i + 1) set_use[i] <= 1;
    end
    else begin
            state <= nxt_state;   
            mem_read <= nxt_mem_read;
            mem_write <= nxt_mem_write;
            mem_addr <= nxt_mem_addr;
            for (i = 0; i < 4; i = i + 1) valid[0][i] <= nxt_valid[0][i];
            for (i = 0; i < 4; i = i + 1) tag[0][i] <= nxt_tag[0][i];
            for (i = 0; i < 4; i = i + 1) dirty[0][i] <= nxt_dirty[0][i];
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[0][i][j] <= nxt_data[0][i][j];
                end
            end
            for (i = 0; i < 4; i = i + 1) valid[1][i] <= nxt_valid[1][i];
            for (i = 0; i < 4; i = i + 1) tag[1][i] <= nxt_tag[1][i];
            for (i = 0; i < 4; i = i + 1) dirty[1][i] <= nxt_dirty[1][i];
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    data[1][i][j] <= nxt_data[1][i][j];
                end
            end
            set_choose <= nxt_set_choose;
            for (i = 0; i < 4; i = i + 1) set_use[i] <= nxt_set_use[i];
    end
end

endmodule
