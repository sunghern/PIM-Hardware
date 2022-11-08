#include <hls_stream.h>
#include <ap_int.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
//#include "header.h"

// bank mode register enable or disable
#define ENABLE_REGISTER 	1
#define DISABLE_REGISTER 	0

// AAM mode enable
#define AAM_DISABLE 0

// splitEngine isjump
#define IS_JUMP 	1
#define NOT_JUMP 	0

// bank number
#define BANK_NUM 10

// bank mode
#define SINGLE_BANK 	1
#define ALL_BANK 		2
#define ALL_BANK_PIM 	3

// crf or bank address register
#define CRF_REG 		0
#define BANK_ADDR_REG 	1

// grf_a_b
#define DATA_GRF_A	1
#define DATA_GRF_B	2

// register - source, destination
#define BANK		0b000
#define GRF_A 	0b001
#define GRF_B 	0b010
#define SRF_A 	0b011
#define SRF_M		0b100

// register local_address
#define SBMR_LOCAL_ADDR			0x3fff
#define ABMR_LOCAL_ADDR			0x3ffe
#define PIM_OP_MODE_LOCAL_ADDR 	0x3ffd
#define CRF_LOCAL_ADDR			0x3ffc
#define GRF_A_B_LOCAL_ADDR		0x3ffb
#define SRF_A_M_LOCAL_ADDR		0x3ffa
#define BANK_ADDR_REG_ADDR		0x3ff9

// opcode - control
#define NOP 	0b0000
#define JUMP 	0b0001
#define EXIT	0b0010
// opcode - move data
#define MOV 	0b0100
#define FILL 	0b0101
// opcode - arithmetic
#define ADD	 	0b1000
#define MUL 	0b1001
#define MAC 	0b1010
#define MAD 	0b1011

#define READ 	0
#define WRITE 	1

#define REAL_ALL_BANK 	0
#define SINGLE_ALL_BANK 1

#define COMPUTE_UNIT_NUM 16

// register for loop num
#define GRF_NUM 8
#define SRF_NUM 4

// bit information
#define COLUMN_BIT		5
#define ROW_BIT			19
#define BANK_GROUP_BIT	2
#define BANK_ADDR_BIT	2
#define CHANNEL_BIT		4
#define GRF_A_IDX_BIT	3
#define GRF_B_IDX_BIT	3
#define ROW_BIT_AAM		18
#define AAM_BIT			1
#define OPCODE_BIT		4
#define SRC_1_NUM_BIT	3
#define SRC_2_NUM_BIT	3
#define SRC_1_BIT			3
#define DEST_BIT			3
#define IMM_1_BIT			11
#define IMM_0_BIT			7
#define IMM_0_MSB_BIT	1
#define SRC_0_NUM_BIT	3
#define DEST_NUM_BIT		3
#define R_BIT				1
#define SRC_0_BIT			3
#define SRC_DEST_BIT		3

#define FLAG_ON 	1
#define FLAG_OFF	0

#define ADD_OP	1
#define BN_OP		2
#define GEMV_OP	3
#define LSTM_OP	4

#define IS_FINISH 	1
#define NOT_FINISH	0

typedef ap_uint<32> stm_b;
typedef ap_uint<16> src_b;
typedef ap_uint<32> interface_b;
typedef ap_uint<32> cmd_b;
typedef ap_uint<32> addr_b;

static uint32_t crf_reg[32];
static uint16_t grf_a_reg[8][16];
static uint16_t grf_b_reg[8][16];
static uint16_t srf_a_reg[8];
static uint16_t srf_m_reg[8];
static uint32_t bank_addr_reg[200];

uint32_t packed_data_count;
uint32_t bank_addr_count;
uint32_t grf_write_count;
bool exit_reg;

union ammAddrBit{
	struct{
		uint32_t 	 		: 4;
		uint32_t  			: 2;
		uint32_t  			: 2;
		uint32_t grf_a_idx 	: GRF_A_IDX_BIT;
		uint32_t grf_b_idx 	: GRF_B_IDX_BIT;
		uint32_t 		: ROW_BIT_AAM;
	};
	uint32_t aam_addr_bit;
};

union checkBit{
	struct{
		uint32_t  			: 15;
		uint32_t aam		: AAM_BIT;
		uint32_t  			: 12;
		uint32_t opcode 	: OPCODE_BIT;
	};
	uint32_t check_instruction;
};

union controlBit{
	struct{
		uint32_t imm_1 		: IMM_1_BIT;
		uint32_t imm_0 		: IMM_0_BIT;
		uint32_t imm_0_msb 	: IMM_0_MSB_BIT;
		uint32_t  			: 9;
		uint32_t opcode	 	: OPCODE_BIT;
	};
	unsigned int contorl_instrcution;
};

union dataBit{
	struct{
		uint32_t  			: 4;
		uint32_t src_0_num 	: SRC_0_NUM_BIT;
		uint32_t  			: 1;
		uint32_t dest_num 	: DEST_NUM_BIT;
		uint32_t  			: 1;
		uint32_t r 			: R_BIT;
		uint32_t  			: 2;
		uint32_t aam		: AAM_BIT;
		uint32_t  			: 6;
		uint32_t src_0 		: SRC_0_BIT;
		uint32_t dest	 	: SRC_DEST_BIT;
		uint32_t opcode 	: OPCODE_BIT;
	};
	uint32_t data_instrcution;
};

union aluBit{
	struct{
		uint32_t src_1_num 	: SRC_1_NUM_BIT;
		uint32_t  			: 1;
		uint32_t src_0_num	: SRC_0_NUM_BIT;
		uint32_t 	 		: 1;
		uint32_t dest_num 	: DEST_NUM_BIT;
		uint32_t  			: 4;
		uint32_t aam		: AAM_BIT;
		uint32_t src_2_num 	: SRC_2_NUM_BIT;
		uint32_t src_1 		: SRC_1_BIT;
		uint32_t src_0 		: SRC_0_BIT;
		uint32_t dest 		: DEST_BIT;
		uint32_t opcode 	: OPCODE_BIT;
	};
	uint32_t alu_instruction;
};

union addrBit{
	struct{
		uint32_t ch 	: CHANNEL_BIT;
		uint32_t ba 	: BANK_ADDR_BIT;
		uint32_t bg 	: BANK_GROUP_BIT;
		uint32_t col 	: COLUMN_BIT;
		uint32_t row 	: ROW_BIT;
	};
	uint32_t addr;
};

union bitSplit{
	struct{
		uint32_t byte_0 : 16;
		uint32_t byte_1 : 16;
	};
	uint32_t full_byte;
};

typedef struct{
	ap_uint<16> operand_0[16];
	ap_uint<16> operand_1[16];
	ap_uint<16> destination_source[16];
	ap_uint<16> mad_srf_a_operand[16];
	ap_uint<16> mad_srf_m_operand[16];

	ap_uint<16> execution_result[16];
}operandSource;

typedef struct{
	ap_uint<32> instruction_reg;
	ap_uint<32> bank_addr_reg;

	ap_uint<4> all_bank_pim_opcode;

	ap_uint<3> operand_source_0;
	ap_uint<3> operand_source_1;
	ap_uint<3> operand_destination;

	ap_uint<3> operand_source_0_num;
	ap_uint<3> operand_source_1_num;
	ap_uint<3> operand_destination_num;
	ap_uint<3> srf_source_num;

	ap_uint<1> aam_mode;

	ap_uint<7> imm_0;
	ap_uint<11> imm_1;
	ap_uint<1> imm_0_msb;

	ap_uint<3> grf_a_modified_idx;
	ap_uint<3> grf_b_modified_idx;
}operandInfo;

typedef struct{
	ap_uint<6> sob;
	ap_uint<6> eob;
	ap_uint<1> jump_flag;

	ap_uint<11> body_size;
	ap_uint<32> loop_size;
	ap_uint<6> jump_program_count;
}jumpInfo;

void memToStream(
		ap_uint<32>* in_r, ap_uint<32> n,
		hls::stream<uint32_t> & strm, addr_b offset){
	for(ap_uint<32> i = offset; i < (n + offset); i++){
#pragma HLS PIPELINE II=1
		strm << in_r[i];
	}
}

void streamToMem(uint32_t* out_r, ap_uint<32> n, hls::stream<uint32_t> & strm){
	for(ap_uint<32> i = 0; i < n; i++){
#pragma HLS PIPELINE II=1
		strm >> out_r[i];
	}
}

void getOperand(
		ap_uint<3> src_bit, ap_uint<3> src_bit_num,
		hls::stream<uint32_t> & strm, addr_b offset,
		src_b *data_from_source, uint32_t *local_addr){
#pragma HLS INLINE off
	bitSplit bs;

	switch(src_bit){
		case GRF_A:{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
				data_from_source[i] = grf_a_reg[src_bit_num][i];
			}
			break;
		}
		case GRF_B:{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
				data_from_source[i] = grf_b_reg[src_bit_num][i];
			}
			break;
		}
		case SRF_A :{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
				data_from_source[i] = srf_a_reg[src_bit_num];
			}
			break;
		}
		case SRF_M :{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
				data_from_source[i] = srf_m_reg[src_bit_num];
			}
			break;
		}
		case BANK:{
			uint32_t tmp[8];
			memToStream(0, 8, strm, offset); // memory to stream
			streamToMem(tmp, 8, strm);

			for(int i = 0; i < (COMPUTE_UNIT_NUM/2); i++){
				bs.full_byte = tmp[i];
				data_from_source[2*i] = bs.byte_0;
				data_from_source[2*i+1] = bs.byte_1;
			}
			break;
		}
	}
}

void writeResult(ap_uint<3> destination, ap_uint<3> destination_num, src_b *execution_result){

	switch(destination){
		case GRF_A:{
			for(int i = 0; i < GRF_NUM; i++){
				grf_a_reg[destination_num][i] = execution_result[i];
			}
			break;
		}
		case GRF_B:{
			for(int i = 0; i < GRF_NUM; i++){
				grf_b_reg[destination_num][i] = execution_result[i];
			}
			break;
		}
	}
}

void intToShort(uint32_t *data_from_host, uint8_t idx, uint8_t a){
	bitSplit bs;

	if(a == DATA_GRF_A){
		for(int i = 0; i < GRF_NUM; i++){
#pragma HLS PIPELINE II=1
			bs.full_byte = data_from_host[i];
			grf_a_reg[idx][2*i] = bs.byte_0;
			grf_a_reg[idx][2*i+1] = bs.byte_1;
		}
	}else if(a == DATA_GRF_B){
		for(int i = 0; i < GRF_NUM; i++){
#pragma HLS PIPELINE II=1
			bs.full_byte = data_from_host[i];
			grf_b_reg[idx][2*i] = bs.byte_0;
			grf_b_reg[idx][2*i+1] = bs.byte_1;
		}
	}else{
		for(int i = 0; i < 4; i++){
#pragma HLS PIPELINE II=1
			bs.full_byte = data_from_host[i];
			srf_a_reg[2*i] = bs.byte_0;
			srf_a_reg[2*i+1] = bs.byte_1;
		}

		for(int i = 0; i < 4; i++){
#pragma HLS PIPELINE II=1
			bs.full_byte = data_from_host[i+4];
			srf_m_reg[2*i] = bs.byte_0;
			srf_m_reg[2*i+1] = bs.byte_1;

		}
	}
}

void dataToReg(uint32_t *data_from_host, uint8_t offset, bool reg){
	if(reg == CRF_REG){
		for(int i = 0; i < 8; i++){
#pragma HLS PIPELINE II=1
			crf_reg[i + offset] = data_from_host[i];
		}
	}else{
		for(int i = 0; i < 8; i++){
#pragma HLS PIPELINE II=1
			bank_addr_reg[i + offset] = data_from_host[i];
		}
	}
}

void writeDataToGrf(
		uint8_t column_local_address, uint32_t *data_from_host){
	switch(column_local_address){
		case 0:{ // write data to grf_a[0]
			intToShort(data_from_host, 0, DATA_GRF_A);
			break;
		}
		case 1:{ // write data to grf_a[1]
			intToShort(data_from_host, 1, DATA_GRF_A);
			break;
		}
		case 2:{ // write data to grf_a[2]
			intToShort(data_from_host, 2, DATA_GRF_A);
			break;
		}
		case 3:{ // write data to grf_a[3]
			intToShort(data_from_host, 3, DATA_GRF_A);
			break;
		}
		case 4:{ // write data to grf_a[4]
			intToShort(data_from_host, 4, DATA_GRF_A);
			break;
		}
		case 5:{ // write data to grf_a[5]
			intToShort(data_from_host, 5, DATA_GRF_A);
			break;
		}
		case 6:{ // write data to grf_a[6]
			intToShort(data_from_host, 6, DATA_GRF_A);
			break;
		}
		case 7:{ // write data to grf_a[7]
			intToShort(data_from_host, 7, DATA_GRF_A);
			break;
		}
		case 8:{ // write data to grf_b[0]
			intToShort(data_from_host, 0, DATA_GRF_B);
			break;
		}
		case 9:{ // write data to grf_b[1]
			intToShort(data_from_host, 1, DATA_GRF_B);
			break;
		}
		case 10:{ // write data to grf_b[2]
			intToShort(data_from_host, 2, DATA_GRF_B);
			break;
		}
		case 11:{ // write data to grf_b[3]
			intToShort(data_from_host, 3, DATA_GRF_B);
			break;
		}
		case 12:{ // write data to grf_b[4]
			intToShort(data_from_host, 4, DATA_GRF_B);
			break;
		}
		case 13:{ // write data to grf_b[5]
			intToShort(data_from_host, 5, DATA_GRF_B);
			break;
		}
		case 14:{ // write data to grf_b[6]
			intToShort(data_from_host, 6, DATA_GRF_B);
			break;
		}
		case 15:{ // write data to grf_b[7]
			intToShort(data_from_host, 7, DATA_GRF_B);
			break;
		}
	}
}

void writeDataToBankReg(
		uint8_t column_local_address, uint32_t *data_from_host){
	switch(column_local_address){
		case 0:{ // write data to bank_addr_reg[0] ~ bank_addr_reg[7]
			dataToReg(data_from_host, 0, BANK_ADDR_REG);
		}break;
		case 1:{ // write data to bank_addr_reg[8] ~ bank_addr_reg[15]
			dataToReg(data_from_host, 8, BANK_ADDR_REG);
		}break;
		case 2:{ // write data to bank_addr_reg[16] ~ bank_addr_reg[23]
			dataToReg(data_from_host, 16, BANK_ADDR_REG);
		}break;
		case 3:{ // write data to bank_addr_reg[24] ~ bank_addr_reg[31]
			dataToReg(data_from_host, 24, BANK_ADDR_REG);
		}break;
		case 4:{ // write data to bank_addr_reg[32] ~ bank_addr_reg[39]
			dataToReg(data_from_host, 32, BANK_ADDR_REG);
		}break;
		case 5:{ // write data to bank_addr_reg[40] ~ bank_addr_reg[47]
			dataToReg(data_from_host, 40, BANK_ADDR_REG);
		}break;
		case 6:{ // write data to bank_addr_reg[48] ~ bank_addr_reg[55]
			dataToReg(data_from_host, 48, BANK_ADDR_REG);
		}break;
		case 7:{ // write data to bank_addr_reg[56] ~ bank_addr_reg[63]
			dataToReg(data_from_host, 56, BANK_ADDR_REG);
		}break;
	}
}

void writeDataToCrf(
		uint8_t column_local_address, uint32_t *data_from_host){
	switch(column_local_address){
		case 0:{ // write data to crf[0] ~ crf[7]
			dataToReg(data_from_host, 0, CRF_REG);
			break;
		}
		case 1:{ // write data to crf[8] ~ crf[15]
			dataToReg(data_from_host, 8, CRF_REG);
			break;
		}
		case 2:{ // write data to crf[16] ~ crf[23]
			dataToReg(data_from_host, 16, CRF_REG);
			break;
		}
		case 3:{ // write data to crf[24] ~ crf[31]
			dataToReg(data_from_host, 24, CRF_REG);
			break;
		}
	}
}

void writeDataToSfr(uint32_t *data_from_host){
	intToShort(data_from_host, 0, 0);
}

void addUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result){
	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a+b;
		execution_result[i] = operand_0[i] + operand_1[i];
	}
}

void mulUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result){
	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a*b;
		execution_result[i] = operand_0[i] * operand_1[i];
	}
}

void macUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result, src_b *destination_source){
	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a*b;
#pragma HLS PIPELINE II=1
		execution_result[i] = destination_source[i] + (operand_0[i] * operand_1[i]);
	}
}

void madUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result){
	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a*b;
#pragma HLS PIPELINE II=1
		execution_result[i] = (operand_0[i] * operand_1[i]) + srf_a_reg[0];
	}
}

void loadEngine(
		operandInfo *op_info_ptr, operandSource *op_source_ptr,
		hls::stream<uint32_t> & strm, uint32_t *local_addr){
#pragma HLS INLINE off

	if((op_info_ptr->all_bank_pim_opcode == MOV) || (op_info_ptr->all_bank_pim_opcode == FILL)){
		// fill operand data to variable
		getOperand(op_info_ptr->operand_source_0, op_info_ptr->operand_source_0_num,
						strm, op_info_ptr->bank_addr_reg, (*op_source_ptr).operand_0, local_addr);
	}else if((op_info_ptr->all_bank_pim_opcode == ADD) ||
			(op_info_ptr->all_bank_pim_opcode == MUL) ||
			(op_info_ptr->all_bank_pim_opcode == MAC) ||
			(op_info_ptr->all_bank_pim_opcode == MAD)){
		// fill operand data to variable - srf number?
		getOperand(op_info_ptr->operand_source_0, op_info_ptr->operand_source_0_num,
				strm, op_info_ptr->bank_addr_reg, (*op_source_ptr).operand_0, local_addr);
		getOperand(op_info_ptr->operand_source_1, op_info_ptr->operand_source_1_num,
				strm, op_info_ptr->bank_addr_reg, (*op_source_ptr).operand_1, local_addr);
		getOperand(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
				strm, op_info_ptr->bank_addr_reg, (*op_source_ptr).destination_source, local_addr);

		getOperand(SRF_A, op_info_ptr->srf_source_num,
				strm, op_info_ptr->bank_addr_reg, (*op_source_ptr).mad_srf_a_operand, local_addr);
	}
}
void fetchInstruction(operandInfo *op_info_ptr, ap_uint<6> *program_count){
	(*op_info_ptr).instruction_reg = crf_reg[*program_count];
	*program_count = *program_count + 1;
}

void jumpEngine(
        operandInfo *op_info_ptr, jumpInfo *jump_info_ptr,
		ap_uint<32> *jump_counter, ap_uint<6> *jump_program_count){
    (*jump_info_ptr).jump_program_count = jump_info_ptr->sob;
    (*jump_info_ptr).body_size = jump_info_ptr->eob - jump_info_ptr->sob + 1;
    (*jump_info_ptr).loop_size = jump_info_ptr->body_size * op_info_ptr->imm_1;
	(*jump_info_ptr).jump_flag = FLAG_ON;
}

unsigned short decodingEngine(operandInfo *op_info_ptr, jumpInfo *jump_info_ptr,
		ap_uint<6> *program_count, ap_uint<32> *jump_counter, ap_uint<6> *jump_program_count){
#pragma HLS INLINE off

	aluBit alu;
	dataBit data;
	controlBit control;
	checkBit check;
	ammAddrBit amm_addr;

	fetchInstruction(op_info_ptr, program_count);

	check.check_instruction = op_info_ptr->instruction_reg;
	(*op_info_ptr).all_bank_pim_opcode = check.opcode; // opcode

	(*op_info_ptr).bank_addr_reg = bank_addr_reg[bank_addr_count++];

	if(check.opcode == JUMP){
		control.contorl_instrcution = op_info_ptr->instruction_reg;
		(*op_info_ptr).imm_1 = control.imm_1;

		*jump_counter = control.imm_1;

		(*jump_info_ptr).eob = *program_count - 2;

		if(control.imm_0_msb){
			(*jump_info_ptr).sob = *program_count - control.imm_0 - 1;
		}else{
			(*jump_info_ptr).sob = *program_count + control.imm_0 - 1;
		}

		jumpEngine(op_info_ptr, jump_info_ptr, jump_counter, jump_program_count);
	}

	if((check.opcode == MOV) || (check.opcode == FILL)){
		data.data_instrcution = op_info_ptr->instruction_reg;
		amm_addr.aam_addr_bit = op_info_ptr->bank_addr_reg;

		// source and destination
		(*op_info_ptr).operand_source_0 = data.src_0;
		(*op_info_ptr).operand_destination = data.dest;

		(*op_info_ptr).operand_source_0_num = data.aam ?
				 ((op_info_ptr->operand_source_0 == GRF_A)?
			     amm_addr.grf_a_idx : amm_addr.grf_b_idx)
				 :data.src_0_num;

		(*op_info_ptr).operand_destination_num = data.aam ?
				((op_info_ptr->operand_destination == GRF_A)?
				amm_addr.grf_a_idx : amm_addr.grf_b_idx)
				:data.dest_num;
	}else{
		alu.alu_instruction = op_info_ptr->instruction_reg;

		// source and destination
		(*op_info_ptr).operand_source_0 = alu.src_0;
		(*op_info_ptr).operand_source_1 = alu.src_1;
		(*op_info_ptr).operand_destination = alu.dest;

		// srf source num
		(*op_info_ptr).srf_source_num = alu.src_2_num;

		// operand source num
		(*op_info_ptr).operand_source_0_num = alu.aam ?
				((op_info_ptr->operand_source_0 == GRF_A)?
				amm_addr.grf_a_idx:amm_addr.grf_b_idx)
				:alu.src_0_num;

		(*op_info_ptr).operand_source_1_num = alu.aam ?
				((op_info_ptr->operand_source_1 == GRF_A)?
				amm_addr.grf_a_idx:amm_addr.grf_b_idx)
				:alu.src_1_num;

		(*op_info_ptr).operand_destination_num = alu.aam ?
				((op_info_ptr->operand_destination == GRF_A)?
				amm_addr.grf_a_idx:amm_addr.grf_b_idx)
				:alu.dest_num;
	}

	return 0;
}

void computeEngine(operandInfo *op_info_ptr, operandSource *op_source_ptr){
#pragma HLS INLINE off
	switch(op_info_ptr->all_bank_pim_opcode){
		// control
		case NOP:{
			// do nothing
			break;
		}
		// change bank mode to all bank mode
		case EXIT:{
			// change bank mode all bank -> all bank pim mode
			exit_reg = ENABLE_REGISTER;
			break;
		}
		// arithmetic
		// a = b+c
		case ADD:{
			addUnit(op_source_ptr->operand_0, op_source_ptr->operand_1,
					(*op_source_ptr).execution_result);

			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num, op_source_ptr->execution_result);
			break;
		}
		// a = b*c
		case MUL:{
			mulUnit(op_source_ptr->operand_0, op_source_ptr->operand_1,
					(*op_source_ptr).execution_result);

			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num, op_source_ptr->execution_result);
			break;
		}
		// a += b*c+d
		case MAD:{

			madUnit(op_source_ptr->operand_0, op_source_ptr->operand_1, (*op_source_ptr).execution_result);

			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num, op_source_ptr->execution_result);
			break;
		}
		// a += b*c
		case MAC:{
			macUnit(op_source_ptr->operand_0, op_source_ptr->operand_1,(*op_source_ptr).execution_result ,op_source_ptr->destination_source);

			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num, op_source_ptr->execution_result);
			break;
		}
		// move data
		case (MOV) | (FILL):{
			// write data to destination
			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num, op_source_ptr->operand_0);
			break;
		}
	}
}

void writeGrfToBram(uint32_t *grf_res, bool is_finish){
	uint32_t tmp_arr[8];

	for(int j = 0; j < 8; j++){
		for(int i = 0; i < 8; i++){
#pragma HLS PIPELINE II=1
			if(is_finish){
				tmp_arr[i] = (grf_b_reg[j][i*2] | (grf_b_reg[j][i*2+1] << 16));
				grf_res[i+j*8+1400] = tmp_arr[i];
			}else{
				tmp_arr[i] = (grf_a_reg[j][i*2] | (grf_a_reg[j][i*2+1] << 16));
				grf_res[i+j*8 + 64*grf_write_count] = tmp_arr[i];
			}
		}
	}

	grf_write_count = grf_write_count + 1;
}

void crfBankAddrCopy(interface_b *ram, ap_uint<1> is_crf){
	if(is_crf){
		for(int i = 0; i < 32; i++){
#pragma HLS PIPELINE II=1
			crf_reg[i] = ram[i];
		}
	}else{
		for(int i = 0; i < 200; i++){
#pragma HLS PIPELINE II=1
			bank_addr_reg[i] = ram[i];
		}
	}
}

void srfCopy(interface_b *ram_a, interface_b *ram_m){
	bitSplit bs;
	for(int i = 0; i < 4; i++){
#pragma HLS PIPELINE II=1
		bs.full_byte = ram_a[i];
		srf_a_reg[2*i] = bs.byte_0;
		srf_a_reg[2*i+1] = bs.byte_1;

		bs.full_byte = ram_m[i+4];
		srf_m_reg[2*i] = bs.byte_0;
		srf_m_reg[2*i+1] = bs.byte_1;
	}
}

void grfCopy(interface_b *ram_a, interface_b *ram_b){
	bitSplit bs;
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < 8; j++){
#pragma HLS PIPELINE II=1
			bs.full_byte = ram_a[i+j];
			grf_a_reg[i][2*j] = bs.byte_0;
			grf_a_reg[i][2*j+1] = bs.byte_1;

			bs.full_byte = ram_b[i+j];
			grf_b_reg[i][2*j] = bs.byte_0;
			grf_b_reg[i][2*j+1] = bs.byte_1;
		}
	}
}

void dataWriteToReg(uint32_t reg_info, uint32_t *data_from_host, uint8_t col){
	if(reg_info == GRF_A_B_LOCAL_ADDR){ // grf
		writeDataToGrf(col, data_from_host);
	}else if(reg_info == SRF_A_M_LOCAL_ADDR){ // srf
		writeDataToSfr(data_from_host);
	}else if(reg_info == BANK_ADDR_REG_ADDR){ // bank_addr_reg
		writeDataToBankReg(col, data_from_host);
	}else if(reg_info == CRF_LOCAL_ADDR){ // crf
		writeDataToCrf(col, data_from_host);
	}
}

void splitPackedData(uint32_t *packed_data, uint32_t *split_data){
	for(int i = 0; i< 8; i++)
#pragma HLS PIPELINE II=1
		split_data[i] = packed_data[i + packed_data_count];

	packed_data_count = packed_data_count + 8;
}

unsigned int pimController(
		uint32_t *local_addr,
		uint32_t packed_data[5000],
		uint32_t grf_res[1500]
		){

#pragma HLS INTERFACE m_axi		port=local_addr	bundle=hbm	offset=slave
#pragma HLS INTERFACE s_axilite	port=packed_data			bundle=ctrl
#pragma HLS INTERFACE s_axilite	port=return	bundle=ctrl

	hls::stream<uint32_t> strm; // read stream
#pragma HLS stream variable=strm	depth=512
#pragma HLS INTERFACE bram port=grf_res
#pragma HLS RESOURCE variable=grf_res core=RAM_1P_BRAM

	uint32_t check_opcode;

 	operandInfo op_info;
 	operandInfo *op_info_ptr;
 	op_info_ptr = &op_info;

 	operandSource op_source;
 	operandSource *op_source_ptr;
 	op_source_ptr = &op_source;

	jumpInfo jump_info;
	jumpInfo *jump_info_ptr;
	jump_info_ptr = &jump_info;

	addrBit addr_bit_info;
	addrBit *addr_bit_info_ptr;
	addr_bit_info_ptr = &addr_bit_info;

	ap_uint<32> result;

	static ap_uint<6> program_count;
	ap_uint<6> jump_program_count;
	ap_uint<32> jump_counter;

	ap_uint<32> full_addr;
	uint32_t split_data[8];

	uint32_t tmp_packed_data_count = 0;
	packed_data_count = 0;

	exit_reg = 0;

	// packed data split
	// opcode check : add or gemv
	check_opcode = packed_data[0];

	// all bank mode change offset  - opcode : 1, all bank mode change : 8
	packed_data_count = packed_data_count + 8 + 1;

	if(check_opcode == GEMV_OP){ // gemv
		// write crf register
		splitPackedData(packed_data, split_data); //17
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 0);

		splitPackedData(packed_data, split_data); // 25
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 1);

		splitPackedData(packed_data, split_data); // 33
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 2);

		splitPackedData(packed_data, split_data); // 41
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 3);

		// write grf register
		splitPackedData(packed_data, split_data); // 49
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 0);

		splitPackedData(packed_data, split_data); // 57
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 1);

		splitPackedData(packed_data, split_data); // 65
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 2);

		splitPackedData(packed_data, split_data); // 73
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 3);

		splitPackedData(packed_data, split_data); // 81
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 4);

		splitPackedData(packed_data, split_data); // 89
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 5);

		splitPackedData(packed_data, split_data); // 97
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 6);

		splitPackedData(packed_data, split_data); // 105
		dataWriteToReg(GRF_A_B_LOCAL_ADDR, split_data, 7);
	}else{ // ADD, BN, LSTM
		splitPackedData(packed_data, split_data);
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 0);

		splitPackedData(packed_data, split_data);
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 1);

		splitPackedData(packed_data, split_data);
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 2);

		splitPackedData(packed_data, split_data);
		dataWriteToReg(CRF_LOCAL_ADDR, split_data, 3);
	}

	uint32_t tmp_ret;
	tmp_ret = crf_reg[0];
	return tmp_ret;

	while(1){
		bank_addr_count = 0;

		if(check_opcode == ADD_OP){ // add
			// write grf_a to result register
			writeGrfToBram(grf_res, NOT_FINISH);

			// end of execution
			if(packed_data_count > 1640){
				// write grf_b to result register
				writeGrfToBram(grf_res, IS_FINISH);
				tmp_packed_data_count = packed_data_count;
				packed_data_count = 0;
				grf_write_count = 0;
				uint32_t tmp_ret;
				tmp_ret = (grf_b_reg[0][0] | (grf_b_reg[0][1] << 16));
				return tmp_ret;
			}

			// write addr register
			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 0);

			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 1);

			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 2);

			// bank mode change -> pim op mode
			splitPackedData(packed_data, split_data);

			// pim execution trigger
			splitPackedData(packed_data, split_data);
		}else if(check_opcode == BN_OP){
			// write grf_a to result register
			writeGrfToBram(grf_res, NOT_FINISH);

			if(packed_data_count > 904){
				// write grf_b result to result register
				writeGrfToBram(grf_res, IS_FINISH);
				tmp_packed_data_count = packed_data_count;
				packed_data_count = 0;
				grf_write_count = 0;
				uint32_t tmp_ret;
				tmp_ret = (grf_b_reg[0][0] | (grf_b_reg[0][1] << 16));
				return tmp_ret;
			}

			// write addr register
			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 0);

			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 1);

			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 2);

			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 3);

			// bank mode change -> pim op mode
			splitPackedData(packed_data, split_data);

			// pim execution trigger
			splitPackedData(packed_data, split_data);
		}else if(check_opcode == GEMV_OP){ // gemv
			if(packed_data_count > 4200){
				// crf - reduce
				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 0);

				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 1);

				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 2);

				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 3);

				// write addr register
				splitPackedData(packed_data, split_data);
				dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 0);

				// bank mode change -> pim op mode
				splitPackedData(packed_data, split_data);

				// pim execution trigger
				splitPackedData(packed_data, split_data);

				// write result to register
				writeGrfToBram(grf_res, NOT_FINISH);
				writeGrfToBram(grf_res, IS_FINISH);
				tmp_packed_data_count = packed_data_count;
				packed_data_count = 0;
				grf_write_count = 0;
				uint32_t tmp_ret;
				tmp_ret = crf_reg[0];
				return tmp_ret;
			}

			// write srf register
			splitPackedData(packed_data, split_data);
			dataWriteToReg(SRF_A_M_LOCAL_ADDR, split_data, 0);

			// write addr register
			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 0);

			// bank mode change -> pim op mode
			splitPackedData(packed_data, split_data);

			// pim execution trigger
			splitPackedData(packed_data, split_data);
		}else if(check_opcode == LSTM_OP){
			if(packed_data_count > 2088){
				// crf - reduce
				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 0);

				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 1);

				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 2);

				splitPackedData(packed_data, split_data);
				dataWriteToReg(CRF_LOCAL_ADDR, split_data, 3);

				// write addr register
				splitPackedData(packed_data, split_data);
				dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 0);

				// bank mode change -> pim op mode
				splitPackedData(packed_data, split_data);

				// pim execution trigger
				splitPackedData(packed_data, split_data);

				// write result to result resiger
				writeGrfToBram(grf_res, NOT_FINISH);
				writeGrfToBram(grf_res, IS_FINISH);
				tmp_packed_data_count = packed_data_count;
				packed_data_count = 0;
				grf_write_count = 0;
				return grf_b_reg[0][0];
			}

			// write srf register
			splitPackedData(packed_data, split_data);
			dataWriteToReg(SRF_A_M_LOCAL_ADDR, split_data, 0);

			// write addr register
			splitPackedData(packed_data, split_data);
			dataWriteToReg(BANK_ADDR_REG_ADDR, split_data, 0);

			// bank mode change -> pim op mode
			splitPackedData(packed_data, split_data);

			// pim execution trigger
			splitPackedData(packed_data, split_data);
		}

		// pim execution start
		for(program_count = 0; program_count < 8;){
	#pragma HLS PIPELINE II=1
			if(jump_info_ptr->jump_flag){
				result=decodingEngine(op_info_ptr, jump_info_ptr, &jump_program_count, &jump_counter, &program_count);
				loadEngine(op_info_ptr, op_source_ptr, strm, local_addr);
				computeEngine(op_info_ptr, op_source_ptr);

				if(jump_program_count == (jump_info_ptr->eob + 1)){
					jump_program_count = jump_info_ptr->sob;
					jump_counter = jump_counter - 1;
				}

				if(jump_counter == 0){
					(*jump_info_ptr).jump_flag = FLAG_OFF;
				}
			}else{
				result=decodingEngine(op_info_ptr, jump_info_ptr, &program_count, &jump_counter, &jump_program_count);
				loadEngine(op_info_ptr, op_source_ptr, strm, local_addr);
				computeEngine(op_info_ptr, op_source_ptr);
			}

			// exit
			if(exit_reg == ENABLE_REGISTER){
				exit_reg = 0;
				break;
			}
		}
	}
}
