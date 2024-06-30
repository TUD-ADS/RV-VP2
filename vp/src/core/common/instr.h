#ifndef RISCV_ISA_INSTR_H
#define RISCV_ISA_INSTR_H

#include <stdint.h>

#include <array>
#include <iostream>

#include "core_defs.h"

namespace Opcode {
// opcode masks used to decode an instruction
enum Parts {
	OP_LUI = 0b0110111,

	OP_AUIPC = 0b0010111,

	OP_JAL = 0b1101111,

	OP_JALR = 0b1100111,
	F3_JALR = 0b000,

	OP_LB = 0b0000011,
	F3_LB = 0b000,
	F3_LH = 0b001,
	F3_LW = 0b010,
	F3_LBU = 0b100,
	F3_LHU = 0b101,
	F3_LWU = 0b110,
	F3_LD = 0b011,

	OP_SB = 0b0100011,
	F3_SB = 0b000,
	F3_SH = 0b001,
	F3_SW = 0b010,
	F3_SD = 0b011,

	OP_BEQ = 0b1100011,
	F3_BEQ = 0b000,
	F3_BNE = 0b001,
	F3_BLT = 0b100,
	F3_BGE = 0b101,
	F3_BLTU = 0b110,
	F3_BGEU = 0b111,

	OP_ADDI = 0b0010011,
	F3_ADDI = 0b000,
	F3_SLTI = 0b010,
	F3_SLTIU = 0b011,
	F3_XORI = 0b100,
	F3_ORI = 0b110,
	F3_ANDI = 0b111,
	F3_SLLI = 0b001,
	F3_SRLI = 0b101,
	F7_SRLI = 0b0000000,
	F7_SRAI = 0b0100000,
	F6_SRLI = 0b000000,  // RV64 special case
	F6_SRAI = 0b010000,  // RV64 special case

	OP_ADD = 0b0110011,
	F3_ADD = 0b000,
	F7_ADD = 0b0000000,
	F3_SUB = 0b000,
	F7_SUB = 0b0100000,
	F3_SLL = 0b001,
	F3_SLT = 0b010,
	F3_SLTU = 0b011,
	F3_XOR = 0b100,
	F3_SRL = 0b101,
	F3_SRA = 0b101,
	F3_OR = 0b110,
	F3_AND = 0b111,

	F3_MUL = 0b000,
	F7_MUL = 0b0000001,
	F3_MULH = 0b001,
	F3_MULHSU = 0b010,
	F3_MULHU = 0b011,
	F3_DIV = 0b100,
	F3_DIVU = 0b101,
	F3_REM = 0b110,
	F3_REMU = 0b111,

	OP_FENCE = 0b0001111,
	F3_FENCE = 0b000,
	F3_FENCE_I = 0b001,

	OP_ECALL = 0b1110011,
	F3_SYS = 0b000,
	F12_ECALL = 0b000000000000,
	F12_EBREAK = 0b000000000001,
	// begin:privileged-instructions
	F12_URET = 0b000000000010,
	F12_SRET = 0b000100000010,
	F12_MRET = 0b001100000010,
	F12_WFI = 0b000100000101,
	F7_SFENCE_VMA = 0b0001001,
	// end:privileged-instructions
	F3_CSRRW = 0b001,
	F3_CSRRS = 0b010,
	F3_CSRRC = 0b011,
	F3_CSRRWI = 0b101,
	F3_CSRRSI = 0b110,
	F3_CSRRCI = 0b111,

	OP_AMO = 0b0101111,
	F5_LR_W = 0b00010,
	F5_SC_W = 0b00011,
	F5_AMOSWAP_W = 0b00001,
	F5_AMOADD_W = 0b00000,
	F5_AMOXOR_W = 0b00100,
	F5_AMOAND_W = 0b01100,
	F5_AMOOR_W = 0b01000,
	F5_AMOMIN_W = 0b10000,
	F5_AMOMAX_W = 0b10100,
	F5_AMOMINU_W = 0b11000,
	F5_AMOMAXU_W = 0b11100,

	F3_AMO_W = 0b010,
	F3_AMO_D = 0b011,

	OP_ADDIW = 0b0011011,
	F3_ADDIW = 0b000,
	F3_SLLIW = 0b001,
	F3_SRLIW = 0b101,
	F7_SRLIW = 0b0000000,
	F7_SRAIW = 0b0100000,

	OP_ADDW = 0b0111011,
	F3_ADDW = 0b000,
	F7_ADDW = 0b0000000,
	F3_SUBW = 0b000,
	F7_SUBW = 0b0100000,
	F3_SLLW = 0b001,
	F3_SRLW = 0b101,
	F7_SRLW = 0b0000000,
	F3_SRAW = 0b101,
	F7_SRAW = 0b0100000,
	F7_MULW = 0b0000001,
	F3_MULW = 0b000,
	F3_DIVW = 0b100,
	F3_DIVUW = 0b101,
	F3_REMW = 0b110,
	F3_REMUW = 0b111,

	// F and D extension
	OP_FMADD_S = 0b1000011,
	F2_FMADD_S = 0b00,
	F2_FMADD_D = 0b01,
	OP_FADD_S = 0b1010011,
	F7_FADD_S = 0b0000000,
	F7_FADD_D = 0b0000001,
	F7_FSUB_S = 0b0000100,
	F7_FSUB_D = 0b0000101,
	F7_FCVT_D_S = 0b0100001,
	F7_FMUL_S = 0b0001000,
	F7_FMUL_D = 0b0001001,
	F7_FDIV_S = 0b0001100,
	F7_FDIV_D = 0b0001101,
	F7_FLE_S = 0b1010000,
	F3_FLE_S = 0b000,
	F3_FLT_S = 0b001,
	F3_FEQ_S = 0b010,
	F7_FSGNJ_D = 0b0010001,
	F3_FSGNJ_D = 0b000,
	F3_FSGNJN_D = 0b001,
	F3_FSGNJX_D = 0b010,
	F7_FMIN_S = 0b0010100,
	F3_FMIN_S = 0b000,
	F3_FMAX_S = 0b001,
	F7_FMIN_D = 0b0010101,
	F3_FMIN_D = 0b000,
	F3_FMAX_D = 0b001,
	F7_FCVT_S_D = 0b0100000,
	F7_FSGNJ_S = 0b0010000,
	F3_FSGNJ_S = 0b000,
	F3_FSGNJN_S = 0b001,
	F3_FSGNJX_S = 0b010,
	F7_FLE_D = 0b1010001,
	F3_FLE_D = 0b000,
	F3_FLT_D = 0b001,
	F3_FEQ_D = 0b010,
	F7_FCVT_S_W = 0b1101000,
	RS2_FCVT_S_W = 0b00000,
	RS2_FCVT_S_WU = 0b00001,
	RS2_FCVT_S_L = 0b00010,
	RS2_FCVT_S_LU = 0b00011,
	F7_FCVT_D_W = 0b1101001,
	RS2_FCVT_D_W = 0b00000,
	RS2_FCVT_D_WU = 0b00001,
	RS2_FCVT_D_L = 0b00010,
	RS2_FCVT_D_LU = 0b00011,
	F7_FCVT_W_D = 0b1100001,
	RS2_FCVT_W_D = 0b00000,
	RS2_FCVT_WU_D = 0b00001,
	RS2_FCVT_L_D = 0b00010,
	RS2_FCVT_LU_D = 0b00011,
	F7_FSQRT_S = 0b0101100,
	F7_FSQRT_D = 0b0101101,
	F7_FCVT_W_S = 0b1100000,
	RS2_FCVT_W_S = 0b00000,
	RS2_FCVT_WU_S = 0b00001,
	RS2_FCVT_L_S = 0b00010,
	RS2_FCVT_LU_S = 0b00011,
	F7_FMV_X_W = 0b1110000,
	F3_FMV_X_W = 0b000,
	F3_FCLASS_S = 0b001,
	F7_FMV_X_D = 0b1110001,
	F3_FMV_X_D = 0b000,
	F3_FCLASS_D = 0b001,
	F7_FMV_W_X = 0b1111000,
	F7_FMV_D_X = 0b1111001,
	OP_FLW = 0b0000111,
	F3_FLW = 0b010,
	F3_FLD = 0b011,
	OP_FSW = 0b0100111,
	F3_FSW = 0b010,
	F3_FSD = 0b011,
	OP_FMSUB_S = 0b1000111,
	F2_FMSUB_S = 0b00,
	F2_FMSUB_D = 0b01,
	OP_FNMSUB_S = 0b1001011,
	F2_FNMSUB_S = 0b00,
	F2_FNMSUB_D = 0b01,
	OP_FNMADD_S = 0b1001111,
	F2_FNMADD_S = 0b00,
	F2_FNMADD_D = 0b01,

	// P standard extension
	// clang-format off
	OP_P 				= 0b1110111,

	F7_ONE_OP			= 0b1010110,
	F7_ONE_OP2 			= 0b1010111,
	
	F7_RADD16 			= 0b0000'000,
	F7_RSUB16 			= 0b0000'001,
	F7_RCRAS16 			= 0b0000'010,
	F7_RCRSA16 			= 0b0000'011,
	F7_RADD8 			= 0b0000'100,
	F7_RSUB8 			= 0b0000'101,
	F7_SCMPLT16			= 0b0000'110,
	F7_SCMPLT8 			= 0b0000'111,
	
	F7_KADD16 			= 0b0001'000,
	F7_KSUB16 			= 0b0001'001,
	F7_KCRAS16 			= 0b0001'010,
	F7_KCRSA16 			= 0b0001'011,
	F7_KADD8 			= 0b0001'100,
	F7_KSUB8 			= 0b0001'101,
	F7_SCMPLE16			= 0b0001'110,
	F7_SCMPLE8 			= 0b0001'111,
	
	F7_URADD16 			= 0b0010'000,
	F7_URSUB16 			= 0b0010'001,
	F7_URCRAS16			= 0b0010'010,
	F7_URCRSA16			= 0b0010'011,
	F7_URADD8 			= 0b0010'100,
	F7_URSUB8			= 0b0010'101,
	F7_UCMPLT16			= 0b0010'110,
	F7_UCMPLT8			= 0b0010'111,
	
	F7_UKADD16 			= 0b0011'000,
	F7_UKSUB16 			= 0b0011'001,
	F7_UKCRAS16 		= 0b0011'010,
	F7_UKCRSA16 		= 0b0011'011,
	F7_UKADD8 			= 0b0011'100,
	F7_UKSUB8 			= 0b0011'101,
	F7_UCMPLE16 		= 0b0011'110,
	F7_UCMPLE8 			= 0b0011'111,
	
	F7_ADD16 			= 0b0100'000,
	F7_SUB16 			= 0b0100'001,
	F7_CRAS16 			= 0b0100'010,
	F7_CRSA16 			= 0b0100'011,
	F7_ADD8 			= 0b0100'100,
	F7_SUB8 			= 0b0100'101,
	F7_CMPEQ16			= 0b0100'110,
	F7_CMPEQ8 			= 0b0100'111,
	
	F7_SRA16 			= 0b0101'000,
	F7_SRL16 			= 0b0101'001,
	F7_SLL16 			= 0b0101'010,
	F7_KSLRA16 			= 0b0101'011,
	F7_SRA8				= 0b0101'100,
	F7_SRL8				= 0b0101'101,
	F7_SLL8				= 0b0101'110,
	F7_KSLRA8			= 0b0101'111,
	
	F7_SRA16_u 			= 0b0110'000,
	F7_SRL16_u 			= 0b0110'001,
	F7_KSLL16			= 0b0110'010,
	F7_KSLRA16_U		= 0b0110'011,
	F7_SRA8_u			= 0b0110'100,
	F7_SRL8_u			= 0b0110'101,
	F7_KSLL8			= 0b0110'110,
	F7_KSLRA8_u			= 0b0110'111,
	
	F7_SRAI16			= 0b0111'000,
	F7_SRAI16_u			= 0b0111'000,
	F7_SRLI16			= 0b0111'001,
	F7_SRLI16_u			= 0b0111'001,
	F7_SLLI16 			= 0b0111'010,
	F7_KSLLI16 			= 0b0111'010,
	F7_SRAI8			= 0b0111'100,
	F7_SRAI8_u			= 0b0111'100,
	F7_SRLI8			= 0b0111'101,
	F7_SRLI8_u			= 0b0111'101,
	F7_SLLI8			= 0b0111'110,
	F7_KSLLI8			= 0b0111'110,

	F7_SMIN16 			= 0b1000'000,
	F7_SMAX16 			= 0b1000'001,
	F7_SCLIP16 			= 0b1000'010,
	F7_UCLIP16 			= 0b1000'010,
	F7_KHM16			= 0b1000'011,
	F7_SMIN8			= 0b1000'100,
	F7_SMAX8			= 0b1000'101,
	F7_SCLIP8			= 0b1000'110,
	F7_UCLIP8 			= 0b1000'110,
	F7_KHM8				= 0b1000'111,

	F7_UMIN16 			= 0b1001'000,
	F7_UMAX16 			= 0b1001'001,
	F7_KHMX16			= 0b1001'011,
	F7_UMIN8			= 0b1001'100,
	F7_UMAX8			= 0b1001'101,
	F7_KHMX8			= 0b1001'111,

	F7_SMUL16 			= 0b1010'000,
	F7_SMULX16 			= 0b1010'001,
	F7_SMUL8			= 0b1010'100,
	F7_SMULX8			= 0b1010'101,

	F7_UMUL16 			= 0b1011'000,
	F7_UMULX16 			= 0b1011'001,
	F7_UMUL8			= 0b1011'100,
	F7_UMULX8			= 0b1011'101,

	
	F7_SMAQA			= 0b1100'100,
	F7_SMAQA_SU			= 0b1100'101,
	F7_UMAQA			= 0b1100'110,
	F7_WEXT				= 0b1100'111,

	F7_WEXTI			= 0b1101'111,

	F7_AVE 				= 0b1110'000,
	F7_SCLIP32 			= 0b1110'010,
	F7_BITREV			= 0b1110'011,
	F7_BITREVI			= 0b1110'100,
	
	F7_UCLIP32 			= 0b1111'010,
	F7_PBSAD			= 0b1111'110,
	F7_PBSADA			= 0b1111'111,

	SUBF5_INSB0 		= 0b00'000,
	SUBF5_INSB1 		= 0b00'001,
	SUBF5_INSB2 		= 0b00'010,
	SUBF5_INSB3			= 0b00'011,

	SUBF5_SUNPKD810		= 0b01'000,
	SUBF5_SUNPKD820		= 0b01'001,
	SUBF5_SUNPKD830		= 0b01'010,
	SUBF5_SUNPKD831		= 0b01'011,
	SUBF5_SUNPKD832		= 0b10'011,
	SUBF5_ZUNPKD810		= 0b01'100,
	SUBF5_ZUNPKD820		= 0b01'101,
	SUBF5_ZUNPKD830		= 0b01'110,
	SUBF5_ZUNPKD831		= 0b01'111,
	SUBF5_ZUNPKD832		= 0b10'111,
	
	SUBF5_KABS8			= 0b10'000,
	SUBF5_KABS16		= 0b10'001,
	SUBF5_KABS32		= 0b10'010,
	SUBF5_SUNPKD83_2	= 0b10'011,
	SUBF5_KABSW 		= 0b10'100,
	SUBF5_ZUNPKD83_2	= 0b10'111,

	SUBF5_SWAP8			= 0b11'000,

	SUBF5_CLRS8			= 0b00'000,
	SUBF5_CLZ8			= 0b00'001,

	SUBF5_CLRS16		= 0b01'000,
	SUBF5_CLZ16		    = 0b01'001,

	SUBF5_CLRS32		= 0b11'000,
	SUBF5_CLZ32	        = 0b11'001,

	F7_KADDW		= 0b0000'000,
	F7_KSUBW		= 0b0000'001,
	F7_KADDH		= 0b0000'010,
	F7_KSUBH		= 0b0000'011,
	F7_SMBB16		= 0b0000'100,
	F7_KDMBB		= 0b0000'101,
	F7_KHMBB		= 0b0000'110,

	F7_UKADDW		= 0b0001'000,
	F7_UKSUBW		= 0b0001'001,
	F7_UKADDH		= 0b0001'010,
	F7_UKSUBH		= 0b0001'011,
	F7_SMBT16		= 0b0001'100,
	F7_KDMBT		= 0b0001'101,
	F7_KHMBT		= 0b0001'110,
	F7_PKBT16		= 0b0001'111,
	
 	F7_RADDW		= 0b0010'000,
	F7_RSUBW		= 0b0010'001,
	F7_SRA_U		= 0b0010'010,
	F7_KSLLW		= 0b0010'011,
	F7_SMTT16		= 0b0010'100,
	F7_KDMTT		= 0b0010'101,
	F7_KHMTT		= 0b0010'110,

 	F7_URADDW		= 0b0011'000,
	F7_URSUBW		= 0b0011'001,
	F7_SRAIW_U		= 0b0011'010,
	F7_KSLLIW		= 0b0011'011,
	F7_KMDA			= 0b0011'100,
	F7_KMXDA		= 0b0011'101,
	F7_PKTB16		= 0b0011'111,

 	F7_SMMUL		= 0b0100'000,
	F7_KMMSB		= 0b0100'001,
	F7_SMMWB		= 0b0100'010,
	F7_KMMAWB		= 0b0100'011,
	F7_KMADA		= 0b0100'100,
	F7_KMAXDA		= 0b0100'101,
	F7_KMSDA		= 0b0100'110,
	F7_KMSXDA		= 0b0100'111,
	
 	F7_SMMUL_U		= 0b0101'000,
	F7_KMMSB_U		= 0b0101'001,
	F7_SMMWB_U		= 0b0101'010,
	F7_KMMAWB_U		= 0b0101'011,
	F7_SMDS			= 0b0101'100,
	F7_KMABB		= 0b0101'101,
	F7_KMADS		= 0b0101'110,
	F7_SMAL			= 0b0101'111,
 	
	F7_KMMAC		= 0b0110'000,
	F7_KWMMUL		= 0b0110'001,
	F7_SMMWT		= 0b0110'010,
	F7_KMMAWT		= 0b0110'011,
	F7_SMDRS		= 0b0110'100,
	F7_KMABT		= 0b0110'101,
	F7_KMADRS		= 0b0110'110,
	F7_KSLRAW		= 0b0110'111,
 	
	F7_KMMAC_U		= 0b0111'000,
	F7_KWMMUL_U		= 0b0111'001,
	F7_SMMWT_U		= 0b0111'010,
	F7_KMMAWT_U		= 0b0111'011,
	F7_SMXDS		= 0b0111'100,
	F7_KMATT		= 0b0111'101,
	F7_KMAXDS		= 0b0111'110,
	F7_KSLRAW_U		= 0b0111'111,
 	
	F7_RADD64		= 0b1000'000,
	F7_RSUB64		= 0b1000'001,
	F7_SMAR64		= 0b1000'010,
	F7_SMSR64		= 0b1000'011,
	F7_SMALBB		= 0b1000'100,
	F7_SMALDS		= 0b1000'101,
	F7_SMALDA		= 0b1000'110,
	F7_KMMWB2		= 0b1000'111,

 	F7_KADD64		= 0b1001'000,
	F7_KSUB64		= 0b1001'001,
	F7_KMAR64		= 0b1001'010,
	F7_KMSR64		= 0b1001'011,
	F7_SMALBT		= 0b1001'100,
	F7_SMALDRS		= 0b1001'101,
	F7_SMALXDA		= 0b1001'110,
	F7_KMMWB2_U		= 0b1001'111,

 	F7_URADD64		= 0b1010'000,
	F7_URSUB64		= 0b1010'001,
	F7_UMAR64		= 0b1010'010,
	F7_UMSR64		= 0b1010'011,
	F7_SMALTT		= 0b1010'100,
	F7_SMALXDS		= 0b1010'101,
	F7_SMSLDA		= 0b1010'110,
	F7_KMMWT2		= 0b1010'111,

 	F7_UKADD64		= 0b1011'000,
	F7_UKSUB64		= 0b1011'001,
	F7_UKMAR64		= 0b1011'010,
	F7_UKMSR64		= 0b1011'011,
	F7_SMSLXDA		= 0b1011'110,
	F7_KMMWT2_U		= 0b1011'111,

 	F7_ADD64		= 0b1100'000,
	F7_SUB64		= 0b1100'001,
	F7_MADDR32		= 0b1100'010,
	F7_MSUBR32		= 0b1100'011,
	F7_KMMAWB2		= 0b1100'111,
 	
	F7_KDMABB		= 0b1101'001,
	F7_SRAI_U		= 0b1101'010,
	F7_KDMABB16		= 0b1101'100,
	F7_KDMBB16		= 0b1101'101,
	F7_KHMBB16		= 0b1101'110,
	F7_KMMAWB2_U	= 0b1101'111,
	
 	F7_MULSR64		= 0b1110'000,
	F7_KDMABT		= 0b1110'001,
	F7_KDMABT16		= 0b1110'100,
	F7_KDMBT16		= 0b1110'101,
	F7_KHMBT16		= 0b1110'110,
	F7_KMMAWT2		= 0b1110'111,

	F7_MULR64		= 0b1111'000,
	F7_KDMATT		= 0b1111'001,
	F7_KDMATT16		= 0b1111'100,
	F7_KDMTT16		= 0b1111'101,
	F7_KHMTT16		= 0b1111'110,
	F7_KMMAWT2_U	= 0b1111'111,

	F7_RADD32		= 0b0000'000,
	F7_RSUB32		= 0b0000'001,
	F7_RCRAS32		= 0b0000'010,
	F7_RCRSA32		= 0b0000'011,

	F7_KADD32		= 0b0001'000,
	F7_KSUB32		= 0b0001'001,
	F7_KCRAS32		= 0b0001'010,
	F7_KCRSA32		= 0b0001'011,
	F7_SMBT32		= 0b0001'100,
	F7_PKBT32		= 0b0001'111,

	F7_URADD32		= 0b0010'000,
	F7_URSUB32		= 0b0010'001,
	F7_URCRAS32		= 0b0010'010,
	F7_URCRSA32		= 0b0010'011,
	F7_SMTT32		= 0b0010'100,
	
	F7_UKADD32		= 0b0011'000,
	F7_UKSUB32		= 0b0011'001,
	F7_UKCRAS32		= 0b0011'010,
	F7_UKCRSA32		= 0b0011'011,
	F7_KMDA32		= 0b0011'100,
	F7_KMXDA32		= 0b0011'101,
	F7_PKTB32		= 0b0011'111,

	F7_ADD32		= 0b0100'000,
	F7_SUB32		= 0b0100'001,
	F7_CRAS32		= 0b0100'010,
	F7_CRSA32		= 0b0100'011,
	F7_KMAXDA3_2	= 0b0100'101,
	F7_KMSDA32		= 0b0100'110,
	F7_KMSXDA3_2	= 0b0100'111,

	F7_SRA32		= 0b0101'000,
	F7_SRL32		= 0b0101'001,
	F7_SLL32		= 0b0101'010,
	F7_KSLRA32		= 0b0101'011,
	F7_SMDS32		= 0b0101'100,
	F7_KMABB32		= 0b0101'101,
	F7_KMADS32		= 0b0101'110,

	F7_SRA32_U		= 0b0110'000,
	F7_SRL32_U		= 0b0110'001,
	F7_KSLL32		= 0b0110'010,
	F7_KSLRA32_U	= 0b0110'011,
	F7_SMDRS32		= 0b0110'100,
	F7_KMABT32		= 0b0110'101,
	F7_KMADRS32		= 0b0110'110,

	F7_SRAI32		= 0b0111'000,
	F7_SRLI32		= 0b0111'001,
	F7_SLLI32		= 0b0111'010,
	F7_SMXDS32		= 0b0111'100,
	F7_KMATT32		= 0b0111'101,
	F7_KMAXDS32		= 0b0111'110,

	F7_SRAI32_U		= 0b1000'000,
	F7_SRLI32_U		= 0b1000'001,
	F7_KSLLI32		= 0b1000'010,

	F7_SMIN32		= 0b1001'000,
	F7_SMAX32		= 0b1001'001,

	F7_UMIN32		= 0b1010'000,
	F7_UMAX32		= 0b1010'001,

	F7_RSTAS32		= 0b1011'001,
	F7_RSTSA32		= 0b1011'010,
	F7_RSTAS16		= 0b1011'011,
	F7_RSTSA16		= 0b1011'100,

	F7_KSTAS32		= 0b1100'001,
	F7_KSTSA32		= 0b1100'010,
	F7_KSTAS16		= 0b1100'011,
	F7_KSTSA16		= 0b1100'100,

	F7_URSTAS32		= 0b1101'001,
	F7_URSTSA32		= 0b1101'010,
	F7_URSTAS16		= 0b1101'011,
	F7_URSTSA16		= 0b1101'100,

	F7_UKSTAS32		= 0b1110'001,
	F7_UKSTSA32		= 0b1110'010,
	F7_UKSTAS16		= 0b1110'011,
	F7_UKSTSA16		= 0b1110'100,

	F7_STAS32		= 0b1111'001,
	F7_STSA32		= 0b1111'010,
	F7_STAS16		= 0b1111'011,
	F7_STSA16		= 0b1111'100,
	// clang-format on

	// reserved opcodes for custom instructions
	OP_CUST1 = 0b0101011,
	OP_CUST0 = 0b0001011,
};

// each instruction is mapped by the decoder to the following mapping
enum Mapping {
	UNDEF = 0,

	// RV32I base instruction set
	LUI = 1,
	AUIPC,
	JAL,
	JALR,
	BEQ,
	BNE,
	BLT,
	BGE,
	BLTU,
	BGEU,
	LB,
	LH,
	LW,
	LBU,
	LHU,
	SB,
	SH,
	SW,
	ADDI,
	SLTI,
	SLTIU,
	XORI,
	ORI,
	ANDI,
	SLLI,
	SRLI,
	SRAI,
	ADD,
	SUB,
	SLL,
	SLT,
	SLTU,
	XOR,
	SRL,
	SRA,
	OR,
	AND,
	FENCE,
	ECALL,
	EBREAK,

	// Zifencei standard extension
	FENCE_I,

	// Zicsr standard extension
	CSRRW,
	CSRRS,
	CSRRC,
	CSRRWI,
	CSRRSI,
	CSRRCI,

	// RV32M standard extension
	MUL,
	MULH,
	MULHSU,
	MULHU,
	DIV,
	DIVU,
	REM,
	REMU,

	// RV32A standard extension
	LR_W,
	SC_W,
	AMOSWAP_W,
	AMOADD_W,
	AMOXOR_W,
	AMOAND_W,
	AMOOR_W,
	AMOMIN_W,
	AMOMAX_W,
	AMOMINU_W,
	AMOMAXU_W,

	// RV64I base integer set (addition to RV32I)
	LWU,
	LD,
	SD,
	ADDIW,
	SLLIW,
	SRLIW,
	SRAIW,
	ADDW,
	SUBW,
	SLLW,
	SRLW,
	SRAW,

	// RV64M standard extension (addition to RV32M)
	MULW,
	DIVW,
	DIVUW,
	REMW,
	REMUW,

	// RV64A standard extension (addition to RV32A)
	LR_D,
	SC_D,
	AMOSWAP_D,
	AMOADD_D,
	AMOXOR_D,
	AMOAND_D,
	AMOOR_D,
	AMOMIN_D,
	AMOMAX_D,
	AMOMINU_D,
	AMOMAXU_D,

	// RV32F standard extension
	FLW,
	FSW,
	FMADD_S,
	FMSUB_S,
	FNMADD_S,
	FNMSUB_S,
	FADD_S,
	FSUB_S,
	FMUL_S,
	FDIV_S,
	FSQRT_S,
	FSGNJ_S,
	FSGNJN_S,
	FSGNJX_S,
	FMIN_S,
	FMAX_S,
	FCVT_W_S,
	FCVT_WU_S,
	FMV_X_W,
	FEQ_S,
	FLT_S,
	FLE_S,
	FCLASS_S,
	FCVT_S_W,
	FCVT_S_WU,
	FMV_W_X,

	// RV64F standard extension (addition to RV32F)
	FCVT_L_S,
	FCVT_LU_S,
	FCVT_S_L,
	FCVT_S_LU,

	// RV32D standard extension
	FLD,
	FSD,
	FMADD_D,
	FMSUB_D,
	FNMSUB_D,
	FNMADD_D,
	FADD_D,
	FSUB_D,
	FMUL_D,
	FDIV_D,
	FSQRT_D,
	FSGNJ_D,
	FSGNJN_D,
	FSGNJX_D,
	FMIN_D,
	FMAX_D,
	FCVT_S_D,
	FCVT_D_S,
	FEQ_D,
	FLT_D,
	FLE_D,
	FCLASS_D,
	FCVT_W_D,
	FCVT_WU_D,
	FCVT_D_W,
	FCVT_D_WU,

	// P standard extension :: 16-bit Addition & Subtraction Instructions
	ADD16,
	RADD16,
	URADD16,
	KADD16,
	UKADD16,
	SUB16,
	RSUB16,
	URSUB16,
	KSUB16,
	UKSUB16,
	CRAS16,
	RCRAS16,
	URCRAS16,
	KCRAS16,
	UKCRAS16,
	CRSA16,
	RCRSA16,
	URCRSA16,
	KCRSA16,
	UKCRSA16,
	STAS16,
	RSTAS16,
	URSTAS16,
	KSTAS16,
	UKSTAS16,
	STSA16,
	RSTSA16,
	URSTSA16,
	KSTSA16,
	UKSTSA16,

	// P standard extension :: 8-bit Addition & Subtraction Instructions
	ADD8,
	RADD8,
	URADD8,
	KADD8,
	UKADD8,
	SUB8,
	RSUB8,
	URSUB8,
	KSUB8,
	UKSUB8,

	// P standard extension :: 16-bit Shift Instructions
	SRA16,
	SRAI16,
	SRA16_u,
	SRAI16_u,
	SRL16,
	SRLI16,
	SRL16_u,
	SRLI16_u,
	SLL16,
	SLLI16,
	KSLL16,
	KSLLI16,
	KSLRA16,
	KSLRA16_u,

	// P standard extension :: 8-bit Shift Instructions
	SRA8,
	SRAI8,
	SRA8_u,
	SRAI8_u,
	SRL8,
	SRLI8,
	SRL8_u,
	SRLI8_u,
	SLL8,
	SLLI8,
	KSLL8,
	KSLLI8,
	KSLRA8,
	KSLRA8_u,

	// P standard extension :: 16-bit Compare Instructions
	CMPEQ16,
	SCMPLT16,
	SCMPLE16,
	UCMPLT16,
	UCMPLE16,

	// P standard extension :: 8-bit Compare Instructions
	CMPEQ8,
	SCMPLT8,
	SCMPLE8,
	UCMPLT8,
	UCMPLE8,

	// P standard extension :: 16-bit Multiply Instructions
	SMUL16,
	SMULX16,
	UMUL16,
	UMULX16,
	KHM16,
	KHMX16,

	// P standard extension :: 8-bit Multiply Instructions
	SMUL8,
	SMULX8,
	UMUL8,
	UMULX8,
	KHM8,
	KHMX8,

	// P standard extension :: 16-bit Misc Instructions
	SMIN16,
	UMIN16,
	SMAX16,
	UMAX16,
	SCLIP16,
	UCLIP16,
	KABS16,
	CLRS16,
	CLZ16,
	SWAP16,

	// P standard extension :: 8-bit Misc Instructions
	SMIN8,
	UMIN8,
	SMAX8,
	UMAX8,
	KABS8,
	SCLIP8,
	UCLIP8,
	CLRS8,
	CLZ8,
	SWAP8,

	// P standard extension :: 8-bit Unpacking Instructions
	SUNPKD810,
	SUNPKD820,
	SUNPKD830,
	SUNPKD831,
	SUNPKD832,
	ZUNPKD810,
	ZUNPKD820,
	ZUNPKD830,
	ZUNPKD831,
	ZUNPKD832,

	// P standard extension :: 16-bit Packing Instructions
	PKBT16,
	PKTB16,

	// P standard extension :: Most Significant Word “32x32” Multiply & Add Instructions
	SMMUL,
	SMMUL_u,
	KMMAC,
	KMMAC_u,
	KMMSB,
	KMMSB_u,
	KWMMUL,
	KWMMUL_u,

	// P standard extension :: Most Significant Word “32x16” Multiply & Add Instructions
	SMMWB,
	SMMWB_u,
	SMMWT,
	SMMWT_u,
	KMMAWB,
	KMMAWB_u,
	KMMAWT,
	KMMAWT_u,
	KMMWB2,
	KMMWB2_u,
	KMMWT2,
	KMMWT2_u,
	KMMAWB2,
	KMMAWB2_u,
	KMMAWT2,
	KMMAWT2_u,

	// P standard extension :: Signed 16-bit Multiply with 32-bit Add/Subtract Instructions
	SMBB16,
	SMBT16,
	SMTT16,
	KMDA,
	KMXDA,
	SMDS,
	SMDRS,
	SMXDS,
	KMABB,
	KMABT,
	KMATT,
	KMADA,
	KMAXDA,
	KMADS,
	KMADRS,
	KMAXDS,
	KMSDA,
	KMSXDA,

	// P standard extension :: Signed 16-bit Multiply with 64-bit Add/Subtract Instructions
	SMAL,

	// P standard extension :: Miscellaneous Instructions
	SCLIP32,
	UCLIP32,
	CLRS32,
	CLZ32,
	PBSAD,
	PBSADA,

	// P standard extension :: 8-bit Multiply with 32-bit Add Instructions
	SMAQA,
	UMAQA,
	SMAQA_SU,

	// P standard extension :: 64-bit Addition & Subtraction Instructions
	ADD64,
	RADD64,
	URADD64,
	KADD64,
	UKADD64,
	SUB64,
	RSUB64,
	URSUB64,
	KSUB64,
	UKSUB64,

	// P standard extension :: 32-bit Multiply with 64-bit Add/Subtract Instructions
	SMAR64,
	SMSR64,
	UMAR64,
	UMSR64,
	KMAR64,
	KMSR64,
	UKMAR64,
	UKMSR64,

	// P standard extension :: Signed 16-bit Multiply with 64-bit Add/Subtract Instructions
	SMALBB,
	SMALBT,
	SMALTT,
	SMALDA,
	SMALXDA,
	SMALDS,
	SMALDRS,
	SMALXDS,
	SMSLDA,
	SMSLXDA,

	// P standard extension :: Q15 saturation instructions
	KADDH,
	KSUBH,
	KHMBB,
	KHMBT,
	KHMTT,
	UKADDH,
	UKSUBH,

	// P standard extension :: Q31 saturation Instructions
	KADDW,
	UKADDW,
	KSUBW,
	UKSUBW,
	KDMBB,
	KDMBT,
	KDMTT,
	KSLRAW,
	KSLRAW_u,
	KSLLW,
	KSLLIW,
	KDMABB,
	KDMABT,
	KDMATT,
	KABSW,

	// P standard extension :: 32-bit Computation Instructions
	RADDW,
	URADDW,
	RSUBW,
	URSUBW,
	MULR64,
	MULSR64,

	// P standard extension :: Overflow/Saturation status manipulation instructions
	RDOV,
	CLROV,

	// P standard extension :: Non-SIMD Miscellaneous Instructions
	AVE,
	SRA_u,
	SRAI_u,
	BITREV,
	BITREVI,
	WEXT,
	WEXTI,
	CMIX,
	INSB,
	MADDR32,
	MSUBR32,
	MAX,
	MIN,

	// RV64D standard extension (addition to RV32D)
	FCVT_L_D,
	FCVT_LU_D,
	FMV_X_D,
	FCVT_D_L,
	FCVT_D_LU,
	FMV_D_X,

	// privileged instructions
	URET,
	SRET,
	MRET,
	WFI,
	SFENCE_VMA,

	NUMBER_OF_INSTRUCTIONS
};

// type denotes the instruction format
enum class Type {
	UNKNOWN = 0,
	R,
	I,
	S,
	B,
	U,
	J,
	R4,
};

extern std::array<const char*, NUMBER_OF_INSTRUCTIONS> mappingStr;
extern std::array<const char*, 32> regnamePrettyStr;

Type getType(Mapping mapping);
}  // namespace Opcode

#define BIT_RANGE(instr, upper, lower) (instr & (((1 << (upper - lower + 1)) - 1) << lower))
#define BIT_SLICE(instr, upper, lower) (BIT_RANGE(instr, upper, lower) >> lower)
#define BIT_SINGLE(instr, pos) (instr & (1 << pos))
#define BIT_SINGLE_P1(instr, pos) (BIT_SINGLE(instr, pos) >> pos)
#define BIT_SINGLE_PN(instr, pos, new_pos) ((BIT_SINGLE(instr, pos) >> pos) << new_pos)
#define EXTRACT_SIGN_BIT(instr, pos, new_pos) ((BIT_SINGLE_P1(instr, pos) << 31) >> (31 - new_pos))

struct Instruction {
	Instruction() : instr(0) {}

	Instruction(uint32_t instr) : instr(instr) {}

	inline uint32_t quadrant() {
		return instr & 0x3;
	}

	inline bool is_compressed() {
		return quadrant() < 3;
	}

	inline uint32_t c_format() {
		return instr & 0xffff;
	}

	inline uint32_t c_opcode() {
		return BIT_SLICE(instr, 15, 13);
	}

	inline uint32_t c_b12() {
		return BIT_SINGLE_P1(instr, 12);
	}

	inline uint32_t c_rd() {
		return rd();
	}

	inline uint32_t c_rd_small() {
		return BIT_SLICE(instr, 9, 7) | 8;
	}

	inline uint32_t c_rs2_small() {
		return BIT_SLICE(instr, 4, 2) | 8;
	}

	inline uint32_t c_rs2() {
		return BIT_SLICE(instr, 6, 2);
	}

	inline uint32_t c_imm() {
		return BIT_SLICE(instr, 6, 2) | EXTRACT_SIGN_BIT(instr, 12, 5);
	}

	inline uint32_t c_uimm() {
		return BIT_SLICE(instr, 6, 2) | (BIT_SINGLE_P1(instr, 12) << 5);
	}

	inline uint32_t c_f2_high() {
		return BIT_SLICE(instr, 11, 10);
	}

	inline uint32_t c_f2_low() {
		return BIT_SLICE(instr, 6, 5);
	}

	Opcode::Mapping decode_normal(Architecture arch);

	Opcode::Mapping decode_and_expand_compressed(Architecture arch);

	inline uint32_t csr() {
		// cast to unsigned to avoid sign extension when shifting
		return BIT_RANGE((uint32_t)instr, 31, 20) >> 20;
	}

	inline uint32_t zimm() {
		return BIT_RANGE(instr, 19, 15) >> 15;
	}

	inline unsigned shamt() {
		return (BIT_RANGE(instr, 25, 20) >> 20);
	}

	inline unsigned shamt_w() {
		return (BIT_RANGE(instr, 24, 20) >> 20);
	}

	inline int32_t funct2() {
		return (BIT_RANGE(instr, 26, 25) >> 25);
	}

	inline int32_t funct3() {
		return (BIT_RANGE(instr, 14, 12) >> 12);
	}

	inline int32_t funct12() {
		// cast to unsigned to avoid sign extension when shifting
		return (BIT_RANGE((uint32_t)instr, 31, 20) >> 20);
	}

	inline int32_t funct7() {
		// cast to unsigned to avoid sign extension when shifting
		return (BIT_RANGE((uint32_t)instr, 31, 25) >> 25);
	}

	inline int32_t funct6() {
		// cast to unsigned to avoid sign extension when shifting
		return (BIT_RANGE((uint32_t)instr, 31, 26) >> 26);
	}

	inline int32_t funct5() {
		// cast to unsigned to avoid sign extension when shifting
		return (BIT_RANGE((uint32_t)instr, 31, 27) >> 27);
	}

	inline uint32_t frm() {
		return BIT_SLICE(instr, 14, 12);
	}

	inline uint32_t fence_succ() {
		return BIT_SLICE(instr, 23, 20);
	}

	inline uint32_t fence_pred() {
		return BIT_SLICE(instr, 27, 24);
	}

	inline uint32_t fence_fm() {
		return BIT_SLICE(instr, 31, 28);
	}

	inline bool aq() {
		return BIT_SINGLE(instr, 26);
	}

	inline bool rl() {
		return BIT_SINGLE(instr, 25);
	}

	inline int32_t opcode() {
		return BIT_RANGE(instr, 6, 0);
	}

	inline int32_t J_imm() {
		return (BIT_SINGLE(instr, 31) >> 11) | BIT_RANGE(instr, 19, 12) | (BIT_SINGLE(instr, 20) >> 9) |
		       (BIT_RANGE(instr, 30, 21) >> 20);
	}

	inline int32_t I_imm() {
		return BIT_RANGE(instr, 31, 20) >> 20;
	}

	inline int32_t S_imm() {
		return (BIT_RANGE(instr, 31, 25) >> 20) | (BIT_RANGE(instr, 11, 7) >> 7);
	}

	inline int32_t B_imm() {
		return (BIT_SINGLE(instr, 31) >> 19) | (BIT_SINGLE(instr, 7) << 4) | (BIT_RANGE(instr, 30, 25) >> 20) |
		       (BIT_RANGE(instr, 11, 8) >> 7);
	}

	inline int32_t U_imm() {
		return BIT_RANGE(instr, 31, 12);
	}

	inline uint32_t rs1() {
		return BIT_RANGE(instr, 19, 15) >> 15;
	}

	inline uint32_t rs2() {
		return BIT_RANGE(instr, 24, 20) >> 20;
	}

	inline uint32_t rs3() {
		return BIT_RANGE((uint32_t)instr, 31, 27) >> 27;
	}

	inline uint32_t rd() {
		return BIT_RANGE(instr, 11, 7) >> 7;
	}

	inline uint32_t data() {
		return instr;
	}

   private:
	// use signed variable to have correct sign extension in immediates
	int32_t instr;
};

#endif  // RISCV_ISA_INSTR_H
