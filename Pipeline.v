`timescale 1ns/1ps


module Pipeline (
    input CLOCK,
    input RESET       // Reset

);
//=========================
// IF
//=========================
wire [31:0] IF_PC_WIRE; 
wire [31:0] IF_NPC_WIRE; 
wire [31:0] IF_INSTRUCTION_WIRE;
wire [31:0] IF_MUX_WIRE;
wire [31:0] IF_ADDER_WIRE;
//========================
// ID
//=========================
wire [31:0] ID_INSTRUCTION_WIRE; // va pal CU
wire [31:0] ID_PC_WIRE;
wire [31:0] ID_TAG_WIRE; //sale del TAG
wire [1:0] REG_IN_PC_SEL_WIRE;
wire [1:0] ID_PC_SEL_WIRE;

// Control UNIT
wire [3:0] CU_ALU_OP_WIRE, CU_SOH_OP_WIRE;
wire CU_LOAD_WIRE;
wire CU_BRANCH_WIRE; 
wire CU_JUMPL_WIRE;
wire CU_RF_LE_WIRE;
wire CU_CALL_WIRE;
wire CU_a;
wire CU_WE_PSR_WIRE; 
wire CU_E_WIRE;
wire CU_RW_WIRE;
wire [1:0] CU_MEM_SIZE_WIRE;

// ID MUX NOP STALL
wire [3:0] STALL_ALU_OP_WIRE, STALL_SOH_OP_WIRE;
wire STALL_LOAD_WIRE;
wire STALL_BRANCH_WIRE; 
wire STALL_JUMPL_WIRE;
wire STALL_RF_LE_WIRE; 
wire STALL_CALL_WIRE;
wire STALL_a;
wire STALL_WE_PSR_WIRE; 
wire STALL_E_WIRE;
wire STALL_RW_WIRE;
wire [1:0] STALL_MEM_SIZE_WIRE;


//ID Data Fowarding MUXes
wire [31:0] DF_PA_WIRE; 
wire [31:0] DF_A_ALU_WIRE; 
wire [31:0] DF_A_MEM_WIRE; 
wire [31:0] DF_A_WB_WIRE;
wire [31:0] DF_A_OUT_WIRE;
wire [1:0]  DF_Sel_A_WIRE;

wire [31:0] DF_PB_WIRE;
wire [31:0] DF_B_ALU_WIRE; 
wire [31:0] DF_B_MEM_WIRE; 
wire [31:0] DF_B_WB_WIRE;
wire [31:0] DF_B_OUT_WIRE;
wire [1:0]  DF_Sel_B_WIRE;

wire [31:0] DF_PC_D_WIRE;
wire [31:0] DF_C_ALU_WIRE; 
wire [31:0] DF_C_MEM_WIRE; 
wire [31:0] DF_C_WB_WIRE;
wire [31:0] DF_C_OUT_WIRE;
wire [1:0]  DF_Sel_C_WIRE;

//ID Instruction fields
wire [4:0] ID_MUX_RD_WIRE; // este es mi out del mux
wire [4:0] ID_RD_WIRE;
wire [4:0] ID_RS1_WIRE;
wire [4:0] ID_RS2_WIRE;
wire [29:0] ID_OFFSET_WIRE;
wire [3:0]  ID_COND_WIRE;
wire [12:0] ID_SIMM13_WIRE;

assign ID_RD_WIRE = ID_INSTRUCTION_WIRE[29:25];
assign ID_RS1_WIRE = ID_INSTRUCTION_WIRE[18:14];
assign ID_RS2_WIRE = ID_INSTRUCTION_WIRE[4:0];
assign ID_OFFSET_WIRE = ID_INSTRUCTION_WIRE[29:0];
assign ID_COND_WIRE = ID_INSTRUCTION_WIRE[28:25];
assign ID_SIMM13_WIRE = ID_INSTRUCTION_WIRE[12:0];

//================================
//EX stage
//================================
wire [31:0] EX_MUX_ALU_CALL;
wire [31:0] EX_PC_WIRE;
wire [31:0] EX_ALU_A_WIRE; 
wire [31:0] EX_SOH_R_WIRE; 
wire [31:0] EX_SOH_N_WIRE;
wire [31:0] EX_SOH_OUT_WIRE; 
wire [31:0] EX_ALU_OUT_WIRE; 
wire [31:0] EX_PC_D_WIRE;
wire [12:0] EX_SIMM13_WIRE;
wire [3:0] EX_ALU_OP_WIRE, EX_SOH_IS_WIRE;
wire [4:0] EX_RD_WIRE;

//salidas de CH
wire [1:0] EX_CH_PC_SEL;
wire [1:0] PC_SEL_WIRE;
wire clr_IF_WIRE;

//señales ICC que van al CH
wire EX_CH_Z_WIRE; 
wire EX_CH_N_WIRE; 
wire EX_CH_V_WIRE;
wire EX_CH_C_WIRE;

//señales ALU que van al MUX ICC
wire EX_ALU_Z_WIRE; 
wire EX_ALU_N_WIRE; 
wire EX_ALU_V_WIRE;
wire EX_ALU_C_WIRE;

//señales PSR que van al MUX ICC
wire EX_PSR_Z_WIRE; 
wire EX_PSR_N_WIRE; 
wire EX_PSR_V_WIRE; 
wire EX_PSR_C_WIRE;

wire EX_WE_PSR_WIRE; 
wire EX_CALL_WIRE;
wire EX_RF_LE_WIRE; 
wire EX_a_WIRE;
wire EX_RW_DM_WIRE; 
wire EX_E_WIRE;
wire EX_LOAD_WIRE;
wire [1:0] EX_MEM_SIZE_WIRE;

//dhdu
wire NOP_STALL_WIRE, DHDU_LE_WIRE;

//=========================
// MEM stage
//=========================
wire [31:0] MEM_DI_WIRE;
wire [1:0] MEM_SIZE_WIRE;
wire MEM_RW_WIRE; 
wire MEM_E_WIRE; 
wire MEM_LOAD_WIRE; 
wire MEM_RF_LE_WIRE;
wire [4:0] MEM_RD_WIRE;
wire [31:0] MEM_ALU_OUT_WIRE, MEM_DM_OUT_WIRE, MEM_MUX_OUT_WIRE; 

//=========================
//wb stage
//=========================
wire [31:0] WB_MUX_OUT_WIRE;
wire [4:0] WB_RD_WIRE;
wire WB_RF_LE_WIRE;




//===================================================================================
//Instanciasde modulos
//==========================

// IF Stage
MUX_IF MUX_IF_0 (
    .npc_in(IF_NPC_WIRE),
    .alu_out(EX_MUX_ALU_CALL),          
    .ta(ID_TAG_WIRE),               
    .sel(PC_SEL_WIRE),              
    .mux_out(IF_MUX_WIRE)          
);
Adder ADDER_0 (
    .mux_out(IF_MUX_WIRE),         
    .adder_out(IF_ADDER_WIRE)
);
NPC_IF NPC_IF_0 (
    .clk(CLOCK),
    .R(RESET),
    .LE(DHDU_LE_WIRE),              
    .adder_out(IF_ADDER_WIRE),        
    .npc(IF_NPC_WIRE)
);
PC_IF PC_IF_0 (
    .clk(CLOCK),
    .R(RESET),
    .LE(DHDU_LE_WIRE),              
    .nPC(IF_MUX_WIRE),
    .pc_out(IF_PC_WIRE)
);
Instruction_Memory INSTRUCTION_MEMORY_0 ( // cambiarlo al otro im
    .pc_out(IF_PC_WIRE),
    .instruction(IF_INSTRUCTION_WIRE)
);
Registro_IF_ID REG_IF_ID_0 ( 
    .clk(CLOCK),
    .R(RESET),
    .LE(DHDU_LE_WIRE),
    .pc_in(IF_PC_WIRE),
    .instruction_in(IF_INSTRUCTION_WIRE),
    .pc_out(ID_PC_WIRE),
    .instruction_out(ID_INSTRUCTION_WIRE)
);
//ID Stage

Target_Address_Generator TAG_ID_0 (
    // .instr(ID_INSTRUCTION_WIRE),
    .PC(ID_PC_WIRE),
    .TA(ID_TAG_WIRE),
    .OFFSET(ID_OFFSET_WIRE),
    .isBRANCH(STALL_BRANCH_WIRE),
    .isCALL(STALL_CALL_WIRE)
);

Register_File RF_ID_0 (
    //in
    .clk(CLOCK),
    .RA(ID_RS1_WIRE), 
    .RB(ID_RS2_WIRE), 
    .RC(ID_RD_WIRE), 

    .PW(WB_MUX_OUT_WIRE), 
    .LE(WB_RF_LE_WIRE), 
    .RW(WB_RD_WIRE), 
    //out
    .PA(DF_PA_WIRE), 
    .PB(DF_PB_WIRE), 
    .PC_D(DF_PC_D_WIRE)
);


CU_ID CU_ID_0 (
    //in
    .instruction(ID_INSTRUCTION_WIRE),

    //out
    .ID_ALU_OP_out(CU_ALU_OP_WIRE),
    .ID_SOH_OP_out(CU_SOH_OP_WIRE),
    .ID_LOAD_out(CU_LOAD_WIRE),
    .ID_BRANCH_out(CU_BRANCH_WIRE),
    .ID_JUMPL_out(CU_JUMP_WIRE),
    .ID_RF_LE_out(CU_RF_LE_WIRE),
    .ID_CALL_out(CU_CALL_WIRE),
    .ID_a_out(CU_a),
    .ID_WE_PSR_out(CU_WE_PSR_WIRE),
    .ID_E_out(CU_E_WIRE),
    .ID_RW_DM_out(CU_RW_WIRE),
    .ID_SIZE_out(CU_MEM_SIZE_WIRE)
);

MUX_DF_PA MUX_DF_A(
    .DF_Sel_A(DF_Sel_A_WIRE),
    .DF_PA(DF_PA_WIRE),
    .DF_A_ALU(DF_A_ALU_WIRE),
    .DF_A_MEM(DF_A_MEM_WIRE),
    .DF_A_WB(DF_A_WB_WIRE),
    .MUX_A_OUT(DF_A_OUT_WIRE)
);

MUX_DF_PB MUX_DF_B(
    .DF_Sel_B(DF_Sel_B_WIRE),
    .DF_PB(DF_PB_WIRE),
    .DF_B_ALU(DF_B_ALU_WIRE),
    .DF_B_MEM(DF_B_MEM_WIRE),
    .DF_B_WB(DF_B_WB_WIRE),
    .MUX_B_OUT(DF_B_OUT_WIRE)
);

MUX_DF_PC_D MUX_DF_C(
    .DF_Sel_C(DF_Sel_C_WIRE),
    .DF_PC_D(DF_PC_D_WIRE),
    .DF_C_ALU(DF_C_ALU_WIRE),
    .DF_C_MEM(DF_C_MEM_WIRE),
    .DF_C_WB(DF_C_WB_WIRE),
    .MUX_C_OUT(DF_C_OUT_WIRE)
);

MUX_ID_STALL MUX_ID_STALL_0 (
    //in
    .ID_MUX_sel(NOP_STALL_WIRE),

    .ID_MUX_ALU_OP_in(CU_ALU_OP_WIRE),
    .ID_MUX_SOH_OP_in(CU_SOH_OP_WIRE),
    .ID_MUX_LOAD_in(CU_LOAD_WIRE),
    .ID_MUX_BRANCH_in(CU_BRANCH_WIRE),
    .ID_MUX_JUMPL_in(CU_JUMPL_WIRE),
    .ID_MUX_RF_LE_in(CU_RF_LE_WIRE),
    .ID_MUX_CALL_in(CU_CALL_WIRE),
    .ID_MUX_WE_PSR_in(CU_WE_PSR_WIRE),
    .ID_MUX_E_in(CU_E_WIRE),
    .ID_MUX_RW_DM_in(CU_RW_WIRE),
    .ID_MUX_SIZE_in(CU_MEM_SIZE_WIRE),
    
    //out 
    .ID_MUX_ALU_OP_out(STALL_ALU_OP_WIRE),
    .ID_MUX_SOH_OP_out(STALL_SOH_OP_WIRE),
    .ID_MUX_LOAD_out(STALL_LOAD_WIRE),
    .ID_MUX_BRANCH_out(STALL_BRANCH_WIRE),
    .ID_MUX_a_out(STALL_a),
    .ID_MUX_RF_LE_out(STALL_RF_LE_WIRE),
    .ID_MUX_CALL_out(STALL_CALL_WIRE),
    .ID_MUX_WE_PSR_out(STALL_WE_PSR_WIRE),
    .ID_MUX_E_out(STALL_E_WIRE),
    .ID_MUX_SIZE_out(STALL_MEM_SIZE_WIRE),
    .ID_MUX_RW_DM_out(STALL_RW_WIRE),
    .ID_MUX_JUMPL_out(STALL_JUMPL_WIRE)


);
Registro_ID_EX REG_ID_EX_0 (
    //in
    .clk(CLOCK),
    .R(RESET),
    //señales de control
    .ID_ALU_OP_in(STALL_ALU_OP_WIRE),
    .ID_SOH_OP_in(STALL_SOH_OP_WIRE),
    .ID_LOAD_in(STALL_LOAD_WIRE),
    .ID_a_in(STALL_a),
    .ID_RF_LE_in(STALL_RF_LE_WIRE),
    .ID_CALL_in(STALL_CALL_WIRE),
    .ID_WE_PSR_in(STALL_WE_PSR_WIRE),
    .ID_E_in(STALL_E_WIRE),
    .ID_SIZE_in(STALL_MEM_SIZE_WIRE),
    .ID_RW_DM_in(STALL_RW_WIRE),

    // operandos
    .DF_A(DF_A_OUT_WIRE),
    .DF_B(DF_B_OUT_WIRE),
    .DF_C(DF_C_OUT_WIRE),

    //registro destino
    .rd_in(ID_MUX_RD_WIRE),

    //pc sel retroalimentado del CH
    .EX_PC_SEL_in(EX_CH_PC_SEL),

    //simm13
    .imm13_in(ID_SIMM13_WIRE),
    .imm13_out(EX_SIMM13_WIRE),

    //out
    .EX_ALU_OP_out(EX_ALU_OP_WIRE),
    .EX_SOH_OP_out(EX_SOH_IS_WIRE),
    .EX_LOAD_out(EX_LOAD_WIRE),
    .EX_a_out(EX_a_WIRE),
    .EX_RF_LE_out(EX_RF_LE_WIRE),
    .EX_CALL_out(EX_CALL_WIRE),
    .EX_WE_PSR_out(EX_WE_PSR_WIRE),
    .EX_E_out(EX_E_WIRE),
    .EX_SIZE_out(EX_MEM_SIZE_WIRE),
    .EX_RW_DM_out(EX_RW_DM_WIRE),
    .A_out(EX_ALU_A_WIRE),
    .B_out(EX_SOH_R_WIRE),
    .C_out(EX_PC_D_WIRE),
    .rd_out(EX_RD_WIRE),
    .EX_PC_SEL_out(PC_SEL_WIRE)
    
);

//EX Stage

Control_Handler Ch_0(
    //in
    .ID_JUMPL(STALL_JUMPL_WIRE),
    .ID_BRANCH(STALL_BRANCH_WIRE),
    .ID_CALL(STALL_CALL_WIRE),
    .a(EX_a_WIRE),
    .ID_COND(ID_COND_WIRE),

    .MUX_N(EX_CH_N_WIRE),
    .MUX_Z(EX_CH_Z_WIRE),
    .MUX_V(EX_CH_V_WIRE),
    .MUX_C(EX_CH_C_WIRE),

    //out
    .PC_SEL(EX_CH_PC_SEL),
    .clr_IF(clr_IF_WIRE)

);

Program_Status_Register PSR_0(
    //in
    .clk(CLOCK),
    .WE_PSR(EX_WE_PSR_WIRE),
    .ALU_Z(EX_ALU_Z_WIRE),
    .ALU_N(EX_ALU_N_WIRE),
    .ALU_V(EX_ALU_V_WIRE),
    .ALU_C(EX_ALU_C_WIRE),

    //out
    .PSR_Z(EX_PSR_Z_WIRE),
    .PSR_N(EX_PSR_N_WIRE),
    .PSR_V(EX_PSR_V_WIRE),
    .PSR_C(EX_PSR_C_WIRE)
);

MUX_EX_ICC MUX_EX_ICC_0(
    //in
    .ALU_Z(EX_ALU_Z_WIRE),
    .ALU_N(EX_ALU_N_WIRE),
    .ALU_V(EX_ALU_V_WIRE),
    .ALU_C(EX_ALU_C_WIRE),

    .PSR_Z(EX_PSR_Z_WIRE),
    .PSR_N(EX_PSR_N_WIRE),
    .PSR_V(EX_PSR_V_WIRE),
    .PSR_C(EX_PSR_C_WIRE),

    .CH_Z(EX_CH_Z_WIRE),
    .CH_N(EX_CH_N_WIRE),
    .CH_V(EX_CH_V_WIRE),
    .CH_C(EX_CH_C_WIRE)

);


Data_Hazard_Detection_Unit DHDU_0(
    //in
    .RA(ID_RS1_WIRE),
    .RB(ID_RS2_WIRE),
    .RC(ID_RD_WIRE),
    .EX_RD(EX_RD_WIRE),
    .MEM_RD(MEM_RD_WIRE),
    .WB_RD(WB_RD_WIRE),
    .EX_LOAD(EX_LOAD_WIRE),
    .EX_RF_LE(EX_RF_LE_WIRE),
    .MEM_RF_LE(MEM_RF_LE_WIRE),
    .WB_RF_LE(WB_RF_LE_WIRE),

    //out
    .LE_IF(DHDU_LE_WIRE),
    .NOP_STALL(NOP_STALL_WIRE),
    .SEL_A(DF_Sel_A_WIRE),
    .SEL_B(DF_Sel_B_WIRE),
    .SEL_C(DF_Sel_C_WIRE)

);

Arithmetic_Logic_Unit ALU_0(
    //in
    .A(EX_ALU_A_WIRE),
    .B(EX_SOH_N_WIRE),
    .OP(EX_ALU_OP_WIRE),
    .Ci(EX_CH_C_WIRE),
    //out
    .Out(EX_ALU_OUT_WIRE),
    .Z(ALU_Z_WIRE),
    .N(ALU_N_WIRE),
    .V(ALU_V_WIRE),
    .C(ALU_C_WIRE)
);

Second_Operand_Handler SOH_0(
    
    .R(EX_SOH_R_WIRE),
    .Imm(EX_SIMM13_WIRE),
    .IS(EX_SOH_IS_WIRE),
    .N(EX_SOH_N_WIRE)
);

MUX_ALU_CALL MUX_ALU_CALL_0(
    //in
    .ALU_OUT(EX_ALU_OUT_WIRE),
    .PC_D(EX_PC_WIRE),
    .EX_CALL(EX_CALL_WIRE),

    //out
    .MUX_OUT(EX_MUX_ALU_CALL)


);

Registro_EX_MEM REG_EX_MEM_0(
    .clk(CLOCK),
    .R(RESET),
    .load_ex(EX_LOAD_WIRE),
    .rf_le_ex(EX_RF_LE_WIRE),
    .E_ex(EX_E_WIRE),
    .Size_ex(EX_MEM_SIZE_WIRE),
    .rw_dm_ex(EX_RW_DM_WIRE),
    .alu_out_ex(),
    .ex_rd(EX_RD_WIRE),
    .PC_D_ex(DF_C_OUT_WIRE),

    //out
    .load_mem(MEM_LOAD_WIRE),
    .rf_le_mem(MEM_RF_LE_WIRE),
    .E_mem(MEM_E_WIRE),
    .size_mem(MEM_SIZE_WIRE),
    .rw_dm_mem(MEM_RW_WIRE),
    .alu_out_mem(MEM_ALU_OUT_WIRE),
    .mem_rd(MEM_RD_WIRE),
    .PC_D_mem(MEM_DI_WIRE)
);

//MEM Stage
 Data_Memory DM_0(
    //in
    .clk(CLOCK),//quitar
    .A_in(MEM_ALU_OUT_WIRE),
    .DI(MEM_DI_WIRE),
    .Size(MEM_SIZE_WIRE),
    .RW(MEM_RW_WIRE),
    .E(MEM_E_WIRE),

    //out
    .DO(MEM_DM_OUT_WIRE)

);

MUX_MEM_OUT MUX_MEM_OUT_0(
    .ALU_OUT(MEM_ALU_OUT_WIRE),
    .MEM_OUT(MEM_DM_OUT_WIRE),
    .MEM_LOAD(MEM_LOAD_WIRE),
    .MUX_OUT(MEM_MUX_OUT_WIRE)

);

//WB Stage
Registro_MEM_WB REG_MEM_WB_0(
    .clk(CLOCK),
    .R(RESET),

    .rf_le_mem(MEM_RF_LE_WIRE),
    .MEM_MUX_OUT(MEM_MUX_OUT_WIRE),
    .mem_rd(MEM_RD_WIRE),

    .rf_le_wb(WB_RF_LE_WIRE),
    .wb_mux_out(WB_MUX_OUT_WIRE),
    .wb_rd(WB_RD_WIRE)
);  


endmodule

module TEST_BENCH4();

endmodule