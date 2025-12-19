`timescale 1ns/1ps

module Pipeline (
    input CLOCK,
    input RESET       // Reset
);
//=========================
// IF
//=========================
wire [31:0] IF_PC_WIRE; 
wire [8:0] IM_A;
assign IM_A = IF_PC_WIRE[8:0];


wire [31:0] NPC_WIRE;
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
wire [31:0] EX_TAG_wire;
wire [1:0] REG_IN_PC_SEL_WIRE;
wire [1:0] ID_PC_SEL_WIRE;

// Control UNIT
wire [3:0] CU_ALU_OP_WIRE, CU_SOH_OP_WIRE;
wire [1:0] CU_LOAD_WIRE;//
wire CU_BRANCH_WIRE; 
wire CU_JUMPL_WIRE;
wire CU_RF_LE_WIRE;
wire CU_CALL_WIRE;
wire CU_a;
wire CU_WE_PSR_WIRE; 
wire CU_E_WIRE;
wire CU_RW_WIRE;
wire [1:0] CU_MEM_SIZE_WIRE;
wire [63:0] CU_Instruction_keyword_wire;
wire CU_SE_WIRE;        // SE generado por CU_ID


// ID MUX NOP STALL
wire [3:0] STALL_ALU_OP_WIRE, STALL_SOH_OP_WIRE;
wire [1:0]STALL_LOAD_WIRE;//
wire STALL_BRANCH_WIRE; 
wire STALL_JUMPL_WIRE;
wire STALL_RF_LE_WIRE; 
wire STALL_CALL_WIRE;
wire STALL_a;
wire STALL_WE_PSR_WIRE; 
wire STALL_E_WIRE;
wire STALL_RW_WIRE;
wire [1:0] STALL_MEM_SIZE_WIRE;
wire STALL_SE_WIRE;     // SE después del MUX_ID_STALL



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
wire [21:0] ID_imm22;
wire ID_bit_i;

assign ID_RD_WIRE = ID_INSTRUCTION_WIRE[29:25];
assign ID_RS1_WIRE = ID_INSTRUCTION_WIRE[18:14];
assign ID_RS2_WIRE = ID_INSTRUCTION_WIRE[4:0];
// assign ID_OFFSET_WIRE = ID_INSTRUCTION_WIRE[29:0];
assign ID_COND_WIRE = ID_INSTRUCTION_WIRE[28:25];
assign ID_SIMM13_WIRE = ID_INSTRUCTION_WIRE[12:0];
// assign ID_imm22 = {{9{ID_SIMM13_WIRE[12]}}, ID_SIMM13_WIRE};
assign CU_a = ID_INSTRUCTION_WIRE[29];
assign ID_bit_i = ID_INSTRUCTION_WIRE[13];

// Crudo de instrucción
wire [29:0] disp30_raw = ID_INSTRUCTION_WIRE[29:0];
wire [21:0] disp22_raw = ID_INSTRUCTION_WIRE[21:0];
wire [21:0] simm13_extended = {{9{ID_SIMM13_WIRE[12]}}, ID_SIMM13_WIRE};
assign ID_imm22 = ID_bit_i
                ? simm13_extended[21:0]
                : disp22_raw;


// Sign-extend
wire signed [31:0] disp30_sext = {{2{disp30_raw[29]}}, disp30_raw};
wire signed [31:0] disp22_sext = {{10{disp22_raw[21]}}, disp22_raw};

// offset en bytes (<<<2)
wire signed [31:0] offset32 =
    CU_CALL_WIRE   ? (disp30_sext <<< 2) :
    CU_BRANCH_WIRE ? (disp22_sext <<< 2) :
                     32'd0;

// Enviar 32 bits completos
assign ID_OFFSET_WIRE = offset32;

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
wire [21:0] EX_IMM22_WIRE;
wire[21:0] ID_IMM_sethi;
wire [21:0] EX_sethi_imm22;
wire [31:0] ex_sethi;

assign ID_IMM_sethi = disp22_raw;
assign ex_sethi = EX_sethi_imm22 << 10;

wire [31:0] EX_CALL_LINK_WIRE;
assign EX_CALL_LINK_WIRE = EX_PC_WIRE;


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
wire [1:0] EX_LOAD_WIRE;
wire [1:0] EX_MEM_SIZE_WIRE;
wire EX_SE_WIRE;                // desde ID/EX hacia EX
wire EX_JUMPL_WIRE;
wire EX_BRANCH_WIRE;
wire [3:0] EX_COND_WIRE;

//dhdu
wire NOP_STALL_WIRE, DHDU_LE_WIRE;
// assign DHDU_LE_WIRE = 1'b1; // deshabilitado para pruebas

wire KILL_ID_WIRE;
assign KILL_ID_WIRE = NOP_STALL_WIRE | (EX_BRANCH_WIRE & clr_IF_WIRE);

//=========================
// MEM stage
//=========================
wire [31:0] MEM_DI_WIRE;
wire [1:0] MEM_SIZE_WIRE;
wire MEM_RW_WIRE; 
wire MEM_E_WIRE; 
wire [31:0] MEM_Sethi_imm22;
wire [1:0] MEM_LOAD_WIRE; 
wire MEM_RF_LE_WIRE;
wire MEM_SE_WIRE;               // desde EX/MEM hacia MEM
wire [4:0] MEM_RD_WIRE;
wire [31:0] MEM_ALU_OUT_WIRE, MEM_DM_OUT_WIRE, MEM_MUX_OUT_WIRE; 
wire [8:0] DM_A;

assign DM_A = MEM_ALU_OUT_WIRE[8:0]; // direccion de memoria

// assign DM_A = MEM_MUX_OUT_WIRE[8:0];
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
    .npc_in(NPC_WIRE),
    .alu_out(EX_MUX_ALU_CALL),          
    .ta(EX_TAG_wire),               
    .sel(EX_CH_PC_SEL),              
    .mux_out(IF_MUX_WIRE)
);
PC_IF PC_IF_0 (
    .clk(CLOCK),
    .R(RESET),
    .LE(DHDU_LE_WIRE),   
    //in           
    .mux_out(IF_MUX_WIRE),
    //out
    .pc_out(IF_PC_WIRE)
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
    //out      
    .npc(NPC_WIRE)
);
Instruction_Memory INSTRUCTION_MEMORY_0 ( // cambiarlo al otro im
    .A(IM_A),
    .I(IF_INSTRUCTION_WIRE)
);
Registro_IF_ID REG_IF_ID_0 ( 
    .clk(CLOCK),
    .R(RESET),
    .LE(DHDU_LE_WIRE),
    .pc_in(IF_PC_WIRE),
    .instruction_in(IF_INSTRUCTION_WIRE),

    .pc_out(ID_PC_WIRE),
    .instruction_out(ID_INSTRUCTION_WIRE),
    .CH_clear(clr_IF_WIRE)
);
//ID Stage

Target_Address_Generator TAG_ID_0 (
    // .instr(ID_INSTRUCTION_WIRE),
    .PC(ID_PC_WIRE),
    .OFFSET(ID_OFFSET_WIRE),
    
    .TA(ID_TAG_WIRE)
);

Register_File RF_ID_0 (
    //in
    .clock(CLOCK),
    .RA(ID_RS1_WIRE), 
    .RB(ID_RS2_WIRE), 
    .RD(ID_RD_WIRE), 

    .PW(WB_MUX_OUT_WIRE), 
    .LE(WB_RF_LE_WIRE), 
    .RW(WB_RD_WIRE), 
    //out
    .PA(DF_PA_WIRE), 
    .PB(DF_PB_WIRE), 
    .PD(DF_PC_D_WIRE)
);

MUX_CALL ID_MUX_CALL0(
    .rd(ID_RD_WIRE),
    .isCALL(STALL_CALL_WIRE),
    .MUX_RD_OUT(ID_MUX_RD_WIRE)

);

CU_ID CU_ID_0 (
    //in
    .instruction(ID_INSTRUCTION_WIRE),

    //out
    .ID_ALU_OP_out(CU_ALU_OP_WIRE),
    .ID_SOH_OP_out(CU_SOH_OP_WIRE),
    .ID_LOAD_out(CU_LOAD_WIRE),
    .ID_BRANCH_out(CU_BRANCH_WIRE),
    .ID_JUMPL_out(CU_JUMPL_WIRE),
    .ID_RF_LE_out(CU_RF_LE_WIRE),
    .ID_CALL_out(CU_CALL_WIRE),
    .ID_a_out(CU_a),
    .ID_WE_PSR_out(CU_WE_PSR_WIRE),
    .ID_E_out(CU_E_WIRE),
    .ID_RW_DM_out(CU_RW_WIRE),
    .ID_SIZE_out(CU_MEM_SIZE_WIRE),
    .keyword(CU_Instruction_keyword_wire),
    .ID_SE_out(CU_SE_WIRE)
);

MUX_DF_PA MUX_DF_A(
    .DF_Sel_A(DF_Sel_A_WIRE),
    .DF_PA(DF_PA_WIRE),
    .DF_A_ALU(EX_MUX_ALU_CALL),
    .DF_A_MEM(MEM_MUX_OUT_WIRE),
    .DF_A_WB(WB_MUX_OUT_WIRE),
    .MUX_A_OUT(DF_A_OUT_WIRE)
);

MUX_DF_PB MUX_DF_B(
    .DF_Sel_B(DF_Sel_B_WIRE),
    .DF_PB(DF_PB_WIRE),
    .DF_B_ALU(EX_MUX_ALU_CALL),
    .DF_B_MEM(MEM_MUX_OUT_WIRE),
    .DF_B_WB(WB_MUX_OUT_WIRE),
    .MUX_B_OUT(DF_B_OUT_WIRE)
);

MUX_DF_PC_D MUX_DF_C(
    .DF_Sel_C(DF_Sel_C_WIRE),
    .DF_PC_D(DF_PC_D_WIRE),
    .DF_C_ALU(EX_MUX_ALU_CALL),
    .DF_C_MEM(MEM_MUX_OUT_WIRE),
    .DF_C_WB(WB_MUX_OUT_WIRE),
    .MUX_C_OUT(DF_C_OUT_WIRE)
);

MUX_ID_STALL MUX_ID_STALL_0 (
    //in
    .ID_MUX_sel(KILL_ID_WIRE),

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
    .ID_MUX_a_in(CU_a),
    .ID_MUX_SE_in(CU_SE_WIRE),
    
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
    .ID_MUX_JUMPL_out(STALL_JUMPL_WIRE),
    .ID_MUX_SE_out(STALL_SE_WIRE)


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
    .ID_SE_in(STALL_SE_WIRE),
    .ID_TAG(ID_TAG_WIRE),
    .EX_PC_SEL_in(EX_CH_PC_SEL),
    .id_sethi_imm(ID_IMM_sethi),
    .ID_PC_in(ID_PC_WIRE),
    .EX_PC_out(EX_PC_WIRE),
    .ID_JUMPL_in(STALL_JUMPL_WIRE),
    .ID_BRANCH_in(STALL_BRANCH_WIRE),
    .ID_COND_in(ID_COND_WIRE),

    // operandos
    .DF_A(DF_A_OUT_WIRE),
    .DF_B(DF_B_OUT_WIRE),
    .DF_C(DF_C_OUT_WIRE),

    //registro destino
    .rd_in(ID_MUX_RD_WIRE),

    //pc sel retroalimentado del CH

    //imm22
    .imm22_in(ID_imm22),
    .imm22_out(EX_IMM22_WIRE),

    //out
    .EX_TAG(EX_TAG_wire),
    .ex_imm_sethi(EX_sethi_imm22),
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
    .EX_SE_out(EX_SE_WIRE),
    .A_out(EX_ALU_A_WIRE),
    .B_out(EX_SOH_R_WIRE),
    .C_out(EX_PC_D_WIRE),
    .rd_out(EX_RD_WIRE),
    .EX_PC_SEL_out(PC_SEL_WIRE),
    .EX_JUMPL_out(EX_JUMPL_WIRE),
    .EX_BRANCH_out(EX_BRANCH_WIRE),
    .EX_COND_out(EX_COND_WIRE)
);

//EX Stage

Control_Handler Ch_0(
    //in
    .ID_JUMPL(EX_JUMPL_WIRE),
    .ID_BRANCH(EX_BRANCH_WIRE),
    .ID_CALL(EX_CALL_WIRE),
    .a(EX_a_WIRE),
    .ID_COND(EX_COND_WIRE),

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

    .EX_WE_PSR(EX_WE_PSR_WIRE),

    .PSR_Z(EX_PSR_Z_WIRE),
    .PSR_N(EX_PSR_N_WIRE),
    .PSR_V(EX_PSR_V_WIRE),
    .PSR_C(EX_PSR_C_WIRE),
    //out
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
    .EX_RF_LE(EX_RF_LE_WIRE),
    .MEM_RF_LE(MEM_RF_LE_WIRE),
    .WB_RF_LE(WB_RF_LE_WIRE),

    //out
    .LE_IF(DHDU_LE_WIRE),
    // .LE_IF(1'b1), // deshabilitado para pruebas
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
    .Ci(EX_ALU_C_WIRE),
    //out
    .Out(EX_ALU_OUT_WIRE),
    .Z(EX_ALU_Z_WIRE),
    .N(EX_ALU_N_WIRE),
    .V(EX_ALU_V_WIRE),
    .C(EX_ALU_C_WIRE)
);

Second_Operand_Handler SOH_0(
    
    .R(EX_SOH_R_WIRE),
    .Imm(EX_IMM22_WIRE),
    .IS(EX_SOH_IS_WIRE),
    .N(EX_SOH_N_WIRE)
);

MUX_ALU_CALL MUX_ALU_CALL_0(
    //in
    .ALU_OUT(EX_ALU_OUT_WIRE),
    .PC_D(EX_CALL_LINK_WIRE),
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
    .size_ex(EX_MEM_SIZE_WIRE),
    .rw_dm_ex(EX_RW_DM_WIRE),
    .se_ex(EX_SE_WIRE),
    .alu_out_ex(EX_MUX_ALU_CALL),
    .ex_sethi_imm22(ex_sethi),
    .ex_rd(EX_RD_WIRE),
    .PC_D_ex(EX_PC_D_WIRE),

    //out
    .mem_sethi_imm22(MEM_Sethi_imm22),
    .load_mem(MEM_LOAD_WIRE),
    .rf_le_mem(MEM_RF_LE_WIRE),
    .E_mem(MEM_E_WIRE),
    .size_mem(MEM_SIZE_WIRE),
    .rw_dm_mem(MEM_RW_WIRE),
    .se_mem(MEM_SE_WIRE),
    .alu_out_mem(MEM_ALU_OUT_WIRE),
    .mem_rd(MEM_RD_WIRE),
    .PC_D_mem(MEM_DI_WIRE)
);

//MEM Stage
 Data_Memory DM_0(
    //in
    .clk(CLOCK),//quitar
    .A_in(DM_A),
    .DI(MEM_DI_WIRE),
    .Size(MEM_SIZE_WIRE),
    .RW(MEM_RW_WIRE),
    .E(MEM_E_WIRE),
    .SE(MEM_SE_WIRE),

    //out
    .DO(MEM_DM_OUT_WIRE)

);

MUX_MEM_OUT MUX_MEM_OUT_0(
    .ALU_OUT(MEM_ALU_OUT_WIRE),
    .MEM_OUT(MEM_DM_OUT_WIRE),
    .MEM_LOAD(MEM_LOAD_WIRE),
    .sethi_imm22(MEM_Sethi_imm22),
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




///////////////////////

// module TB_DEBUGG_FILE();
    
//     reg CLOCK;
//     reg RESET;

//     // Instancia del pipeline
//     Pipeline pipeline (
//         .CLOCK(CLOCK),
//         .RESET(RESET)
//     );

//     // Cargar Instrucciones
//     reg [7:0] instr_bytes [0:511];
//     initial begin

//     // LIMPIAR INSTRUCTION MEMORY y DATA MEM
//     $display("Limpiando Instruction y Data Memory..");
//     for (integer i = 0; i < 512; i = i + 1) begin
//         pipeline.INSTRUCTION_MEMORY_0.imem[i] = 32'b0;
//         pipeline.DM_0.mem[i] = 8'h00;
//     end

//     // CARGAR INSTRUCTION MEMORY
//     $display("=== Cargando debugging_code_SPARC.txt ===");

//     //leer debugging code
//     $readmemb("test/debugging_code_SPARC.txt", instr_bytes);

//     // cargamos IM y DM con las mismas instrucciones
//     for (integer i = 0; i < 511; i = i + 1) begin
//         pipeline.INSTRUCTION_MEMORY_0.imem[i] = instr_bytes[i];
//         pipeline.DM_0.mem[i] = instr_bytes[i];
//         $display("IM[%0d] = %b", i, instr_bytes[i]);
//     end

//     $display("=== Instruction Memory cargada ===\n");

//     end

//     initial begin
//         CLOCK = 0;
//         forever #2 CLOCK = ~CLOCK; // periodo = 4ns
//     end

//     initial begin
//         RESET = 1;
//         #3 RESET = 0;
//     end

//     initial begin
//         #80;
//         $finish;
//     end

//     initial begin
//         #76;
//        $display("Word in address 56 at T=76: %b %b %b %b",
//             pipeline.DM_0.mem[56],
//             pipeline.DM_0.mem[57],
//             pipeline.DM_0.mem[58],
//             pipeline.DM_0.mem[59]
//         );
//     end
//     always @(posedge CLOCK) begin
//             $display("PC=%0d | r5=%0d r6=%0d r16=%0d r17=%0d r18=%0d",
//                 pipeline.IF_PC_WIRE,
//                 pipeline.RF_ID_0.q5,
//                 pipeline.RF_ID_0.q6,
//                 pipeline.RF_ID_0.q16,
//                 pipeline.RF_ID_0.q17,
//                 pipeline.RF_ID_0.q18);
            
//     end

// endmodule

/////////

// module TB_1();
     
//     reg CLOCK;
//     reg RESET;

//     // Instancia del pipeline
//     Pipeline pipeline (
//         .CLOCK(CLOCK),
//         .RESET(RESET)
//     );

//     // Cargar Instrucciones
//     reg [7:0] instr_bytes [0:511];
//     initial begin

//     // LIMPIAR INSTRUCTION MEMORY
//     $display("Limpiando Instruction y Data Memory..");
//     for (integer i = 0; i < 512; i = i + 1) begin
//         pipeline.INSTRUCTION_MEMORY_0.imem[i] = 32'b0;
//         pipeline.DM_0.mem[i] = 8'h00;
//     end

//     // CARGAR INSTRUCTION MEMORY
//     $display("=== Cargando testcode1_SPARC.txt ===");

//     //leer debugging code
//     $readmemb("test/testcode_sparc1.txt", instr_bytes);

//     for (integer i = 0; i < 511; i = i + 1) begin
//         pipeline.INSTRUCTION_MEMORY_0.imem[i] = instr_bytes[i];
//         pipeline.DM_0.mem[i] = instr_bytes[i];
//         $display("IM[%0d] = %b", i, instr_bytes[i]);
//     end

//     $display("=== Instruction and DATA Memory cargada ===\n");

//     end

//     initial begin
//         CLOCK = 0;
//         forever #2 CLOCK = ~CLOCK; // periodo = 4ns
//     end

//     initial begin
//         RESET = 1;
//         #3 RESET = 0;
//     end

//     initial begin
//         #164;
//         $finish;
//     end

//     initial begin
//        #160 begin
//         $display("Word at address 44 at T=160: %b %b %b %b",
//             pipeline.DM_0.mem[44],
//             pipeline.DM_0.mem[45],
//             pipeline.DM_0.mem[46],
//             pipeline.DM_0.mem[47]
//         );
//         end
//     end

//     always @(posedge CLOCK) begin
//             $display("PC=%0d | r1=%0d r2=%0d r3=%0d r5=%0d",
//                 pipeline.IF_PC_WIRE,
//                 pipeline.RF_ID_0.q1,
//                 pipeline.RF_ID_0.q2,
//                 pipeline.RF_ID_0.q3,
//                 pipeline.RF_ID_0.q5);

//             // $display("Word at address 44: %b %b %b %b",
//             //     pipeline.DM_0.mem[44],
//             //     pipeline.DM_0.mem[45],
//             //     pipeline.DM_0.mem[46],
//             //     pipeline.DM_0.mem[47]
//             // );
//     end

// endmodule

module TB_2();
     
    reg CLOCK;
    reg RESET;

    // Instancia del pipeline
    Pipeline pipeline (
        .CLOCK(CLOCK),
        .RESET(RESET)
    );

    // Cargar Instrucciones
    reg [7:0] instr_bytes [0:511];
    initial begin

    // LIMPIAR INSTRUCTION MEMORY
    $display("Limpiando Instruction y Data Memory..");
    for (integer i = 0; i < 512; i = i + 1) begin
        pipeline.INSTRUCTION_MEMORY_0.imem[i] = 32'b0;
        pipeline.DM_0.mem[i] = 8'h00;
    end

    // CARGAR INSTRUCTION MEMORY
    $display("=== Cargando testcode2_SPARC.txt ===");

    //leer debugging code
    $readmemb("test/testcode_sparc2.txt", instr_bytes);

    for (integer i = 0; i < 511; i = i + 1) begin
        pipeline.INSTRUCTION_MEMORY_0.imem[i] = instr_bytes[i];
         pipeline.DM_0.mem[i] = instr_bytes[i];
        $display("IM[%0d] = %b", i, instr_bytes[i]);
    end

    $display("=== Instruction and data Memory cargada ===\n");

    end

    initial begin
        CLOCK = 0;
        forever #2 CLOCK = ~CLOCK; // periodo = 4ns
    end

    initial begin
        RESET = 1;
        #3 RESET = 0;
    end

    initial begin
        #244;
        $finish;
    end
        integer addr;
        integer count;

    initial begin
    #240;
    $display("Contenido de las localizaciones de la 224 a la 263 (bytes):");

    
    count = 0;

    for (addr = 224; addr <= 263; addr = addr + 1) begin

        // imprimir el byte como 8 bits
        $write("%08b ", pipeline.DM_0.mem[addr]);

        count = count + 1;

        // cada 4 bytes, salto de línea
        if (count == 4) begin
            $write("\n");
            count = 0;
        end
    end

    // si no cerró exacto, agrega newline final
    if (count != 0)
        $write("\n");
end


    always @(posedge CLOCK) begin
            $display("PC=%0d | r1=%0d r2=%0d r3=%0d r4=%0d r5=%0d r8=%0d r10=%0d r11=%0d r12=%0d r15=%0d",
                pipeline.IF_PC_WIRE,
                pipeline.RF_ID_0.q1,
                pipeline.RF_ID_0.q2,
                pipeline.RF_ID_0.q3,
                pipeline.RF_ID_0.q4,
                pipeline.RF_ID_0.q5,
                pipeline.RF_ID_0.q8,
                pipeline.RF_ID_0.q10,
                pipeline.RF_ID_0.q11,
                pipeline.RF_ID_0.q12,
                pipeline.RF_ID_0.q15);
    end

endmodule

// module TB4();

//     reg CLOCK;
//     reg RESET;

//     // Instancia del pipeline
//     Pipeline pipeline (
//         .CLOCK(CLOCK),
//         .RESET(RESET)
//     );

//     // Cargar Instrucciones
//     reg [7:0] instr_bytes [0:511];
//     initial begin

//     // LIMPIAR INSTRUCTION MEMORY
//     $display("Limpiando Instruction y Data Memory..");
//     for (integer i = 0; i < 512; i = i + 1) begin
//         pipeline.INSTRUCTION_MEMORY_0.imem[i] = 32'b0;
//         pipeline.DM_0.mem[i] = 8'h00;
//     end

    
//     // CARGAR INSTRUCTION MEMORY
//     $display("=== Cargando debugging_code_SPARC.txt ===");

//     //leer debugging code
//     // $readmemb("test/debugging_code_SPARC.txt", instr_bytes);

//     //leer testcode1
//     // $readmemb("test/testcode_sparc1.txt", instr_bytes);

//     //leer testcode2 (56)
//     $readmemb("test/testcode_sparc2.txt", instr_bytes);

//     //leer testcode2

//     for (integer i = 0; i < 511; i = i + 1) begin
//         pipeline.INSTRUCTION_MEMORY_0.imem[i] = instr_bytes[i];
//         pipeline.DM_0.mem[i] = instr_bytes[i];
//         // $display("IM[%0d] = %b", i, instr_bytes[i]);
//     end

//     $display("=== Instruction Memory cargada ===\n");

//     end

//     // ================================
//     // 2. Clock
//     // ================================
//     initial begin
//         CLOCK = 0;
//         forever #2 CLOCK = ~CLOCK; // periodo = 4ns
//     end

//     initial begin
//         RESET = 1;
//         #3 RESET = 0;
//     end

    


// integer cycle = 0;

// always @(posedge CLOCK) begin
//     cycle = cycle + 1;


//     $display("\n====================== CYCLE %0d ======================", cycle);

//         $display("Word in address 220: %0d %b %b %b",
//                 $signed(pipeline.DM_0.mem[220]),
//                 pipeline.DM_0.mem[221],
//                 pipeline.DM_0.mem[222],
//                 pipeline.DM_0.mem[223]
//             );
//         $display("Byte 224 = %b %b %b %b",
//             pipeline.DM_0.mem[224],
//             pipeline.DM_0.mem[225],
//             pipeline.DM_0.mem[226],
//             pipeline.DM_0.mem[227]
//             );
//         $display("Byte 228 = %b %b %b %b",
//             pipeline.DM_0.mem[228],
//             pipeline.DM_0.mem[229],
//             pipeline.DM_0.mem[230],
//             pipeline.DM_0.mem[231]
//             );
//             $display("Byte 232 = %b %b %b %b",
//             pipeline.DM_0.mem[232],
//             pipeline.DM_0.mem[233],
//             pipeline.DM_0.mem[234],
//             pipeline.DM_0.mem[235]
//             );
//             $display("Byte 236 = %b %b %b %b",
//             pipeline.DM_0.mem[236],
//             pipeline.DM_0.mem[237],
//             pipeline.DM_0.mem[238],
//             pipeline.DM_0.mem[239]
//             );
//             $display("Byte 240 = %b %b %b %b",
//             pipeline.DM_0.mem[240],
//             pipeline.DM_0.mem[241],
//             pipeline.DM_0.mem[242],
//             pipeline.DM_0.mem[243]
//             );
//             $display("Byte 244 = %b %b %b %b",
//             pipeline.DM_0.mem[244],
//             pipeline.DM_0.mem[245],
//             pipeline.DM_0.mem[246],
//             pipeline.DM_0.mem[247]
//             );
//              $display("Byte 248 = %b %b %b %b",
//             pipeline.DM_0.mem[248],
//             pipeline.DM_0.mem[249],
//             pipeline.DM_0.mem[250],
//             pipeline.DM_0.mem[251]
//             );
//              $display("Byte 252 = %b %b %b %b",
//             pipeline.DM_0.mem[252],
//             pipeline.DM_0.mem[253],
//             pipeline.DM_0.mem[254],
//             pipeline.DM_0.mem[255]
//             );
//              $display("Byte 256 = %b %b %b %b",
//             pipeline.DM_0.mem[256],
//             pipeline.DM_0.mem[257],
//             pipeline.DM_0.mem[258],
//             pipeline.DM_0.mem[259]
//             );
//             $display("Byte 260 = %b %b %b %b",
//             pipeline.DM_0.mem[260],
//             pipeline.DM_0.mem[261],
//             pipeline.DM_0.mem[262],
//             pipeline.DM_0.mem[263]
//             );


//             $display("IF STAGE ----------------------------------------------");
//             $display("IF | PC=%0d  NPC=%0d  IR=%b",
//                  pipeline.IF_PC_WIRE,
//                  pipeline.NPC_WIRE,
//                  pipeline.IF_INSTRUCTION_WIRE);
        
//             $display("IF_MUX | EX_PC_SEL in = %b  TA = %b  ALU_OUT = %b  NPC = %b mux_out=%0d",
//                     pipeline.PC_SEL_WIRE,
//                     pipeline.EX_TAG_wire,
//                     pipeline.EX_ALU_OUT_WIRE,
//                     pipeline.NPC_WIRE, 
//                     pipeline.IF_MUX_WIRE);
//             $display("IF_IM | addres_in=%b instrucion_out=%b",
//                     pipeline.IM_A,
//                     pipeline.IF_INSTRUCTION_WIRE);
//             $display("\n");


//         // ID
//         $display("ID_STAGE --------------------------------------------------------------");
//         $display("IF_ID_Pipeline_REG", 
//                  " | PC_in=%0d  IR_in=%b  => PC_out=%0d  IR_out=%b DHDU_LE=%b",
//                  pipeline.IF_PC_WIRE,
//                  pipeline.IF_INSTRUCTION_WIRE,
//                  pipeline.ID_PC_WIRE,
//                  pipeline.ID_INSTRUCTION_WIRE,
//                  pipeline.DHDU_LE_WIRE
//         );
//         $display("ID | RF: RS1=%d RS2=%d RD=%0d  cond=%b  imm22=%b",
//                  pipeline.ID_RS1_WIRE,
//                  pipeline.ID_RS2_WIRE,
//                  pipeline.ID_RD_WIRE,
//                  pipeline.ID_COND_WIRE,
//                  pipeline.ID_imm22);

//         $display("ID | CU: ALU_OP=%b SOH_OP=%b LOAD=%b BR=%b JUMPL=%b RF_LE=%b a=%b WE_PSR=%b CALL=%b SIZE=%b bit_i=%b keyword=%s SE_DM=%b",
//                  pipeline.CU_ALU_OP_WIRE,
//                  pipeline.CU_SOH_OP_WIRE,
//                  pipeline.CU_LOAD_WIRE,
//                  pipeline.CU_BRANCH_WIRE,
//                  pipeline.CU_JUMPL_WIRE, 
//                  pipeline.CU_RF_LE_WIRE,
//                  pipeline.CU_a,
//                  pipeline.CU_WE_PSR_WIRE,  // ojo al nombre del wire aquí
//                  pipeline.CU_CALL_WIRE,
//                  pipeline.CU_MEM_SIZE_WIRE,
//                  pipeline.ID_bit_i,
//                  pipeline.CU_Instruction_keyword_wire,
//                  pipeline.CU_SE_WIRE);

//         $display("CU debug: IR=%b OP=%b OP3=%b OP2=%b",
//          pipeline.ID_INSTRUCTION_WIRE,
//          pipeline.ID_INSTRUCTION_WIRE[31:30],
//          pipeline.ID_INSTRUCTION_WIRE[24:19],
//          pipeline.ID_INSTRUCTION_WIRE[24:22]
// );



//         // $display("ID | RD_INS=%0d MUX_RD=%0d",
//         //          pipeline.ID_RD_WIRE,
//         //          pipeline.ID_MUX_RD_WIRE
//         // );
//         $display("ID | TAG: PC=%d OFFSET=%b isBranch=%b isCALL=%b TA_OUT=%d ",
//             pipeline.ID_PC_WIRE,
//             pipeline.ID_OFFSET_WIRE,
//             pipeline.STALL_BRANCH_WIRE,
//             pipeline.STALL_CALL_WIRE,
//             pipeline.ID_TAG_WIRE
//         );
//         $display("FWD A MUX | PA=%0d  A_ALU=%0d A_MEM=%0d A_WB=%0d | SEL_A=%b | A_OUT=%0d",
//             pipeline.DF_PA_WIRE,
//             pipeline.EX_MUX_ALU_CALL,
//             pipeline.MEM_MUX_OUT_WIRE,
//             pipeline.WB_MUX_OUT_WIRE,
//             pipeline.DF_Sel_A_WIRE,
//             pipeline.DF_A_OUT_WIRE
//         );
//           $display("FWD B MUX | PB=%0d  B_ALU=%0d B_MEM=%0d B_WB=%0d | SEL_B=%b | B_OUT=%0d",
//             pipeline.DF_PB_WIRE,
//             pipeline.EX_MUX_ALU_CALL,
//             pipeline.MEM_MUX_OUT_WIRE,
//             pipeline.WB_MUX_OUT_WIRE,
//             pipeline.DF_Sel_B_WIRE,
//             pipeline.DF_B_OUT_WIRE
//         );
//         $display("FWD C MUX | PC=%0d  C_ALU=%0d C_MEM=%0d C_WB=%0d | SEL_C=%b | C_OUT=%0d",
//             pipeline.DF_PC_D_WIRE,
//             pipeline.EX_MUX_ALU_CALL,
//             pipeline.MEM_MUX_OUT_WIRE,
//             pipeline.WB_MUX_OUT_WIRE,
//             pipeline.DF_Sel_C_WIRE,
//             pipeline.DF_C_OUT_WIRE
//         );

//         $display("ID_EX_PIPELINE_REG_IN ALU_OP=%b SOH_OP=%b LOAD=%b RF_LE=%b a=%b WE_PSR=%b CALL=%b E=%b SIZE=%b RW=%b RD=%0d IMM22=%b DF_A=%0d DF_B=%0d DF_C=%0d SE=%b TA_in=%0d CH_SEL=%b",
//             pipeline.STALL_ALU_OP_WIRE,
//             pipeline.STALL_SOH_OP_WIRE,
//             pipeline.STALL_LOAD_WIRE,
//             pipeline.STALL_RF_LE_WIRE,
//             pipeline.STALL_a,
//             pipeline.STALL_WE_PSR_WIRE,
//             pipeline.STALL_CALL_WIRE,
//             pipeline.STALL_E_WIRE,
//             pipeline.STALL_MEM_SIZE_WIRE,
//             pipeline.STALL_RW_WIRE,
//             pipeline.ID_MUX_RD_WIRE,
//             pipeline.ID_imm22,
//             pipeline.DF_A_OUT_WIRE,
//             pipeline.DF_B_OUT_WIRE,
//             pipeline.DF_C_OUT_WIRE,
//             pipeline.STALL_SE_WIRE,
//             pipeline.ID_TAG_WIRE,
//             pipeline.EX_CH_PC_SEL
//         );
       
//         $display("\n");

//         $display("EX STAGE ---------------------------------------------------------------------------------------------------------------");
//         $display("ID_EX_PIPELINE_REG_OUT ALU_OP=%b SOH_OP=%b LOAD=%b RF_LE=%b a=%b WE_PSR=%b CALL=%b E=%b SIZE=%b RW=%b RD=%0d IMM22=%b A=%0d B=%0d PC_D=%0d PC_SEL=%b SE=%b TA=%0d",
//             pipeline.EX_ALU_OP_WIRE,
//             pipeline.EX_SOH_IS_WIRE,
//             pipeline.EX_LOAD_WIRE,
//             pipeline.EX_RF_LE_WIRE,
//             pipeline.EX_a_WIRE,
//             pipeline.EX_WE_PSR_WIRE,
//             pipeline.EX_CALL_WIRE,
//             pipeline.EX_E_WIRE,
//             pipeline.EX_MEM_SIZE_WIRE,
//             pipeline.EX_RW_DM_WIRE,
//             pipeline.EX_RD_WIRE,
//             pipeline.EX_IMM22_WIRE,
//             pipeline.EX_ALU_A_WIRE,
//             pipeline.EX_SOH_R_WIRE,
//             pipeline.EX_PC_D_WIRE,
//             pipeline.PC_SEL_WIRE,
//             pipeline.EX_SE_WIRE,
//             pipeline.EX_TAG_wire
//         );

//         $display("DHDU | IF_LE =%b ID_RS1=%0d ID_RS2=%0d EX_RD=%0d MEM_RD=%0d WB_RD=%0d | EX_RF_LE=%b MEM_RF_LE=%b WB_RF_LE=%b | SEL_A=%b SEL_B=%b",
//             pipeline.DHDU_LE_WIRE,
//             pipeline.ID_RS1_WIRE,
//             pipeline.ID_RS2_WIRE,
//             pipeline.EX_RD_WIRE,
//             pipeline.MEM_RD_WIRE,
//             pipeline.WB_RD_WIRE,
//             pipeline.EX_RF_LE_WIRE,
//             pipeline.MEM_RF_LE_WIRE,
//             pipeline.WB_RF_LE_WIRE,
//             pipeline.DF_Sel_A_WIRE,
//             pipeline.DF_Sel_B_WIRE
//         );


//         $display("ALU: ALU_A=%0d ALU_B=%0d ALU_OUT=%0d ALU_Z=%b ALU_N=%b ALU_V=%b ALU_C=%b",
//                 $signed(pipeline.EX_ALU_A_WIRE),
//                 $signed(pipeline.EX_SOH_N_WIRE),
//                 $signed(pipeline.EX_ALU_OUT_WIRE),
//                 pipeline.EX_ALU_Z_WIRE,
//                 pipeline.EX_ALU_N_WIRE,
//                 pipeline.EX_ALU_V_WIRE,
//                 pipeline.EX_ALU_C_WIRE
//                );

//         $display("PSR_in: EX_WE_IN=%b ALU_Z=%b ALU_N=%b ALU_V=%b ALU_C=%b ",
//                 pipeline.EX_WE_PSR_WIRE,
//                 pipeline.EX_ALU_Z_WIRE,
//                 pipeline.EX_ALU_N_WIRE,
//                 pipeline.EX_ALU_V_WIRE,
//                 pipeline.EX_ALU_C_WIRE);

//         $display("PSR_out: PSR_Z=%b PSR_N=%b PSR_V=%b PSR_C=%b",
//                 pipeline.EX_PSR_Z_WIRE,
//                 pipeline.EX_PSR_N_WIRE,
//                 pipeline.EX_PSR_V_WIRE,
//                 pipeline.EX_PSR_C_WIRE);

//         $display("MUX_EX_ICC_out: EX_WE=%b MUX_Z=%b MUX_N=%b MUX_V=%b MUX_C=%b", 
//                 pipeline.EX_WE_PSR_WIRE,
//                 pipeline.EX_CH_Z_WIRE,
//                 pipeline.EX_CH_N_WIRE,
//                 pipeline.EX_CH_V_WIRE,
//                 pipeline.EX_CH_C_WIRE,
//                 );

//         $display("CH: PC_SEL_out=%b | BRANCH_in_id=%b CALL_in_id=%b JUMPL_in_id=%b clear_if=%b Z=%b N=%b V=%b C=%b",
//                 pipeline.EX_CH_PC_SEL,
//                 pipeline.STALL_BRANCH_WIRE,
//                 pipeline.STALL_CALL_WIRE,
//                 pipeline.STALL_JUMPL_WIRE,
//                 pipeline.clr_IF_WIRE,
//                 pipeline.EX_CH_Z_WIRE, 
//                 pipeline.EX_CH_N_WIRE, 
//                 pipeline.EX_CH_V_WIRE,
//                 pipeline.EX_CH_C_WIRE);
//         $display("MUX_ALU_CALL | alu_out=%b pc=%b iscall=%b muxout=%b",
//                 pipeline.EX_ALU_OUT_WIRE,
//                 pipeline.EX_PC_D_WIRE,
//                 pipeline.EX_CALL_WIRE,
//                 pipeline.EX_MUX_ALU_CALL
//                 );

//         $display("EX_MEM_PIPELINE_REG_IN | load=%b size=%b e=%b rf=%b rw=%b alu=%0d rd=%d pc=%0d se=%b", 
//                 pipeline.EX_LOAD_WIRE, 
//                 pipeline.EX_MEM_SIZE_WIRE, 
//                 pipeline.EX_E_WIRE,
//                 pipeline.EX_RF_LE_WIRE, 
//                 pipeline.EX_RW_DM_WIRE,
//                 pipeline.EX_MUX_ALU_CALL, 
//                 pipeline.EX_RD_WIRE, 
//                 pipeline.EX_PC_D_WIRE,
//                 pipeline.EX_SE_WIRE);
//         $display("EX_IMM22<<10=%h",
//             pipeline.ex_sethi);
//          $display("\n");

//         $display("MEM STAGE -------------------------------------------------------------------------");
//         $display("EX_MEM_PIPELINE_REG_OUT | load=%b size=%b e=%b rf=%b rw=%b alu=%b rd=%d pc=%b se=%b", 
//          pipeline.MEM_LOAD_WIRE, 
//          pipeline.MEM_SIZE_WIRE, 
//          pipeline.MEM_E_WIRE,
//          pipeline.MEM_RF_LE_WIRE, 
//          pipeline.MEM_RW_WIRE,  
//          pipeline.MEM_ALU_OUT_WIRE,
//          pipeline.MEM_RD_WIRE,
//          pipeline.MEM_DI_WIRE,
//          pipeline.MEM_SE_WIRE
//         );

//         $display("MEM DM: | Address(ALU)=%0d DI(PC)=%0d DO=%b RW=%b E=%b SIZE=%b, SE=%b",
//                 pipeline.DM_A,
//                 pipeline.MEM_DI_WIRE,
//                 pipeline.MEM_DM_OUT_WIRE,
//                 pipeline.MEM_RW_WIRE,
//                 pipeline.MEM_E_WIRE,
//                 pipeline.MEM_SIZE_WIRE,
//                 pipeline.MEM_SE_WIRE);

//         $display("MEM | EX/MEM_ALU_OUT=%0d MUX_MEM_OUT=%0d IMM22=%b RD=%0d",
//                 pipeline.MEM_ALU_OUT_WIRE,
//                 pipeline.MEM_MUX_OUT_WIRE,
//                 pipeline.MEM_Sethi_imm22,
//                 pipeline.MEM_RD_WIRE
//         );

//         $display("MEM_WB_PIPELINE_REG_IN | RF_LE=%b MUX_OUT=%0d RD=%0d",
//             pipeline.MEM_RF_LE_WIRE,
//             pipeline.MEM_MUX_OUT_WIRE,
//             pipeline.MEM_RD_WIRE
//         ); 
        
//         $display("WB STAGE --------------------------------------------------------------");
//         $display("MEM_WB_PIPELINE_REG_OUT | RF_LE=%b MUX_OUT=%0d RD=%0d",
//             pipeline.WB_RF_LE_WIRE,
//             pipeline.WB_MUX_OUT_WIRE,
//             pipeline.WB_RD_WIRE);       
//             //wb         
//         $display("WB | RF_LE=%b  PD=%b  RD=%b",
//                 pipeline.WB_RF_LE_WIRE,
//                 pipeline.WB_MUX_OUT_WIRE,
//                 pipeline.WB_RD_WIRE
//         );

//             // // REGISTER FILE STATE oara debug
//             $display("\n");
//             $display("registros---------------------------------------------------------");
//            $display("PC=%0d | r1=%0d r2=%0d r3=%0d r4=%0d r5=%0d r8=%0d r10=%0d r11=%0d r12=%0d r15=%0d DM[220]=%0d",
//                 pipeline.IF_PC_WIRE,
//                 pipeline.RF_ID_0.q1,
//                 pipeline.RF_ID_0.q2,
//                 $signed(pipeline.RF_ID_0.q3),
//                 pipeline.RF_ID_0.q4,
//                 pipeline.RF_ID_0.q5,
//                 pipeline.RF_ID_0.q8,
//                 pipeline.RF_ID_0.q10,
//                 pipeline.RF_ID_0.q11,
//                 pipeline.RF_ID_0.q12,
//                 pipeline.RF_ID_0.q15,
//                 $signed(pipeline.DM_0.mem[220]));
            

// //             //register file para testcode1
// //             $display("RF | r0=%0d r1=%0d r2=%0d r3=%0d r4=%0d r5=%0d r6=%0d r7=%0d",
// //                 pipeline.RF_ID_0.registers[0],
// //                 pipeline.RF_ID_0.registers[1],
// //                 pipeline.RF_ID_0.registers[2],
// //                 $signed(pipeline.RF_ID_0.registers[3]),
// //                 pipeline.RF_ID_0.registers[4],
// //                 pipeline.RF_ID_0.registers[5],
// //                 pipeline.RF_ID_0.registers[6],
// //                 pipeline.RF_ID_0.registers[7]);
// //             $display("RF | r8=%0d r9=%0d r10=%0d r11=%0d r12=%0d r13=%0d r14=%0d r15=%0d",
// //                 pipeline.RF_ID_0.registers[8],
// //                 pipeline.RF_ID_0.registers[9],
// //                 pipeline.RF_ID_0.registers[10],
// //                 pipeline.RF_ID_0.registers[11],
// //                 pipeline.RF_ID_0.registers[12],
// //                 pipeline.RF_ID_0.registers[13],
// //                 pipeline.RF_ID_0.registers[14],
// //                 pipeline.RF_ID_0.registers[15]);

//         // initial begin
//         //     #76;
//         //     $display("Tiempo 76: ...");
//         // end

//         //debug file
//         // if ($time == 76) begin
//         //     $display("Word at address 56 = %b", pipeline.DM_0.mem[56]);
//         //     $display("\n");
//         // end

//         // //test1
//         // if ($time == 160) begin
//         //     $display("Word at address 44 = %b", pipeline.DM_0.mem[44]);
//         // $display("\n");
//         // end

//         // //test2
//         // if ($time == 240) begin
//         //     $display("Word at address 44 = %b", pipeline.DM_0.mem[224]);
//         //     $display("Word at address 44 = %b", pipeline.DM_0.mem[224]);
//         // $display("\n");
//         // end



//     $display("===========================================================\n");
//     end




//     // Stop debug
//     initial begin
//         // #76 $display("Word at address 56 = %b", pipeline.DM_0.mem[56]);
//         #244 $finish;
//     end
//     // test 1
//     // initial begin
//     //     #164 $finish;
//     // end
//     // //test 2
//     // initial begin
//     //     #244 $finish;
//     // end



// endmodule