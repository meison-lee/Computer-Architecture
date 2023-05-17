module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,
    input                       i_rst_n,

    input                       i_valid,
    input [DATA_W - 1 : 0]      i_A,
    input [DATA_W - 1 : 0]      i_B,
    input [         2 : 0]      i_inst,

    output [2*DATA_W - 1 : 0]   o_data,
    output                      o_done
);
// Do not Modify the above part !!!

// Parameters
    // Definition of states
    parameter S_IDLE = 4'd0;
    parameter S_ADD = 4'd1;
    parameter S_SUB = 4'd2;
    parameter S_AND = 4'd3;
    parameter S_OR  = 4'd4;
    parameter S_SLT = 4'd5;
    parameter S_SRA = 4'd6;
    parameter S_MUL = 4'd7;
    parameter S_DIV = 4'd8;
    parameter S_OUT = 4'd9;

// Wires & Regs
    // Todo
    reg  [3:0] state, next_state;
    reg [2*DATA_W - 1 : 0] alu_out_reg, shift_reg, temp;
    reg [4:0] count_reg;
    // wire [DATA_W - 1 : 0] i_A, i_B;


// Wire Assignments
    // Todo
    assign o_done = (state == S_OUT);
    assign o_data = alu_out_reg;

// Always Combination
    // Todo: FSM
        always @(*) begin
            case(state)
                S_IDLE  :begin

                    if (i_valid)begin
                        next_state = i_inst+1;
                        // $display("after add i_A = %b, i_B = %b", i_A,i_B);
                    end
                    else begin
                        next_state = S_IDLE;
                    end
                end
                S_ADD   :begin

                    next_state = S_OUT;
                end
                S_SUB   :begin

                    next_state = S_OUT;
                    // $display("after add i_A = %b, i_B = %b, out = %b", i_A,i_B, alu_out_reg);
                end
                S_AND   :begin

                    next_state = S_OUT;
                end
                S_OR    :begin

                    next_state = S_OUT;
                end
                S_SLT   :begin

                    next_state = S_OUT;
                end
                S_SRA   :begin

                    next_state = S_OUT;
                end
                S_MUL   :begin
                    if (count_reg == 31)begin
                        next_state = S_OUT;
                    end
                    else begin
                        next_state = S_MUL;
                    end
                end
                S_DIV   :begin
                    if (count_reg == 31)begin
                        next_state = S_OUT;
                    end
                    else begin
                        next_state = S_DIV;
                    end
                end
                S_OUT   :begin
                    next_state = S_IDLE;
                end
                default :begin
                    next_state = S_IDLE;
                end
            endcase
        end
    // Todo: Counter
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            count_reg <= 0;
        end
        else begin
            count_reg <= (state == S_MUL || state == S_DIV) ? count_reg + 1 : count_reg;
        end
    end

    // Todo: ALU output
    always @(posedge o_done)begin
        if (count_reg == 31)begin
            alu_out_reg = shift_reg;
        end
        else begin
            case(state)
                S_ADD   :begin
                    alu_out_reg = $signed(i_A) + $signed(i_B);
                    if (i_A[DATA_W-1] == i_B[DATA_W-1] )begin
                        if (i_A[DATA_W-1] == 1 && (alu_out_reg[DATA_W-1] == 0))begin
                            alu_out_reg = 32'h80000000;
                        end
                        else if (i_A[DATA_W-1] == 0 && (alu_out_reg[DATA_W-1] == 1))begin
                            alu_out_reg = 32'h7fffffff;
                        end
                    end
                    alu_out_reg = {32'b0,alu_out_reg[DATA_W-1:0]};
                end
                S_SUB   :begin
                    alu_out_reg = $signed(i_A) - $signed(i_B);
                    if (i_A[DATA_W-1] != i_B[DATA_W-1] )begin
                        if (i_A[DATA_W-1] == 1 && (alu_out_reg[DATA_W-1] == 0))begin
                            alu_out_reg = 32'h80000000;
                        end
                        else if (i_A[DATA_W-1] == 0 && (alu_out_reg[DATA_W-1] == 1))begin
                            alu_out_reg = 32'h7fffffff;
                        end

                    end
                    alu_out_reg = {32'b0,alu_out_reg[DATA_W-1:0]};
                end
                S_AND   :begin
                    alu_out_reg = i_A & i_B;
                end
                S_OR    :begin
                    alu_out_reg = i_A | i_B;
                end
                S_SLT   :begin
                    alu_out_reg = $signed(i_A) < $signed(i_B);
                end
                S_SRA   :begin
                    alu_out_reg = $signed(i_A) >> $signed(i_B);
                    alu_out_reg = {32'b0,alu_out_reg[DATA_W-1:0]};
                end
            endcase
        end

    end

    // Todo: Shift register
    always @(posedge i_clk)begin
        if (count_reg < 32)begin
            if (state == S_MUL )begin
                if (count_reg == 0)begin
                    shift_reg = {32'b0,i_A};
                end
                // $display("MUL shift_reg = %b, count_reg = %b", shift_reg, count_reg);
                if (shift_reg[0] == 1)begin
                    temp = shift_reg[2 * DATA_W -1:DATA_W] + i_B;
                    shift_reg = {temp[DATA_W:0], shift_reg[DATA_W-1: 1]};
                end
                else begin
                    shift_reg = shift_reg >> 1;
                end
            end
            else if (state == S_DIV)begin
                if (count_reg == 0)begin
                    shift_reg = {32'b0,i_A};
                end
                // $display("DIV shift_reg = %b, count_reg = %b", shift_reg, count_reg);
                if (shift_reg[2 * DATA_W -1:DATA_W] > i_B)begin
                    shift_reg = {shift_reg[2 * DATA_W -1:DATA_W] - i_B,shift_reg[DATA_W-1:0]};
                    shift_reg = shift_reg << 1;
                    shift_reg = {shift_reg[2 *DATA_W -1:1], 1'b1};
                end
                else begin
                    shift_reg = shift_reg << 1;
                end
                if (count_reg == 31)begin
                    if (shift_reg[2 * DATA_W -1:DATA_W] > i_B )begin
                        shift_reg = {shift_reg[2 * DATA_W -1:DATA_W], shift_reg[DATA_W-1:0] << 1};
                        shift_reg = {shift_reg[2 * DATA_W -1:DATA_W] - i_B,shift_reg[DATA_W-1:1], 1'b1};
                    end
                    else begin
                        shift_reg = {shift_reg[2 * DATA_W -1:DATA_W], shift_reg[DATA_W-1:0] << 1};
                    end
                    // $display("DIV shift_reg = %b, count_reg = %b after", shift_reg, count_reg);
                end
            end
        end
    end

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state <= S_IDLE;
        end
        else begin
            state <= next_state;
        end
    end

endmodule