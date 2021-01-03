
//-------------------------------------------------
//--1-read-or-write ram with byte enables ---------
//-------------------------------------------------
module rf_be_1rw #(parameter DEPTH=8, WIDTH=32) (
    output logic[WIDTH-1:0]         dout,
    input  logic                    eph1,
    input  logic                    write,
    input  logic[$clog2(DEPTH)-1:0] addr,
    input  logic[WIDTH/8-1:0]       wben,
    input  logic[WIDTH-1:0]         din
);

localparam NBYTES = WIDTH/8;
localparam DLG2   = $clog2(DEPTH);


logic[NBYTES-1:0][7:0] RAM        [DEPTH-1:0];
logic[NBYTES-1:0][7:0] din_bytes;

assign din_bytes = din;

for (genvar d=0; d<DEPTH; d+=1) begin: g_rf
    for (genvar b=0; b<NBYTES; b+=1) begin : g_bytes
        rregs_en #(8) rgdata ( RAM[d][b], din_bytes[b], eph1, write & wben[b] & addr == d );
    end : g_bytes
end : g_rf

logic[DLG2-1:0] addr_stg;
rregs #(DLG2) rgaddrstg (addr_stg, addr, eph1 );

always_comb begin
    dout = '1;
    for (int i=0; i<DEPTH; i+=1) begin
        dout &= (addr_stg == i) ? RAM[i] : '1;
    end
end

endmodule: rf_be_1rw 
