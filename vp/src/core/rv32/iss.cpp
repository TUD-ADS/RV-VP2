#include "iss.h"

// to save *cout* format setting, see *ISS::show*
#include <boost/io/ios_state.hpp>
// for safe down-cast
#include <boost/lexical_cast.hpp>

using namespace rv32;

#define RAISE_ILLEGAL_INSTRUCTION() raise_trap(EXC_ILLEGAL_INSTR, instr.data());

#define REQUIRE_ISA(X)        \
	if (!(csrs.misa.reg & X)) \
	RAISE_ILLEGAL_INSTRUCTION()

#define RD instr.rd()
#define RS1 instr.rs1()
#define RS2 instr.rs2()
#define RS3 instr.rs3()

const char *regnames[] = {
    "zero (x0)", "ra   (x1)", "sp   (x2)", "gp   (x3)", "tp   (x4)", "t0   (x5)", "t1   (x6)", "t2   (x7)",
    "s0/fp(x8)", "s1   (x9)", "a0  (x10)", "a1  (x11)", "a2  (x12)", "a3  (x13)", "a4  (x14)", "a5  (x15)",
    "a6  (x16)", "a7  (x17)", "s2  (x18)", "s3  (x19)", "s4  (x20)", "s5  (x21)", "s6  (x22)", "s7  (x23)",
    "s8  (x24)", "s9  (x25)", "s10 (x26)", "s11 (x27)", "t3  (x28)", "t4  (x29)", "t5  (x30)", "t6  (x31)",
};

int regcolors[] = {
#if defined(COLOR_THEME_DARK)
    0,  1,  2,  3,  4,  5,  6,  52, 8,  9,  53, 54, 55, 56, 57, 58,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
#elif defined(COLOR_THEME_LIGHT)
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 153, 154, 155, 156, 157, 158,
    116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131,
#else
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#endif
};

template <typename T, int64_t lo = std::numeric_limits<T>::min(), int64_t hi = std::numeric_limits<T>::max()>
static T sat_add(bool *out_ov, int64_t a, int64_t b) {
	if (a > 0 && b > 0 && hi - a < b) {
		*out_ov = true;
		return hi;
	}
	if (a < 0 && b < 0 && lo - a > b) {
		*out_ov = true;
		return lo;
	}
	return static_cast<T>(a + b);
}

template <typename T, int64_t lo = std::numeric_limits<T>::min(), int64_t hi = std::numeric_limits<T>::max()>
static T sat_sub(bool *out_ov, int64_t a, int64_t b) {
	if ((b > 0 && a < lo + b) || (b < 0 && a > hi + b)) {
		*out_ov = true;
		return lo;
	}
	if ((b > 0 && a > hi - b) || (b < 0 && a < lo - b)) {
		*out_ov = true;
		return hi;
	}
	return static_cast<T>(a - b);
}

template <typename T, uint64_t hi = std::numeric_limits<T>::max()>
static T sat_uadd(bool *out_ov, uint64_t a, uint64_t b) {
	if (hi - a < b) {
		*out_ov = true;
		return hi;
	}
	return static_cast<T>(a + b);
}

template <typename T, uint64_t hi = std::numeric_limits<T>::max()>
static T sat_usub(bool *out_ov, uint64_t a, uint64_t b) {
	if (a < b) {
		*out_ov = true;
		return 0;
	}
	if (a > hi - b) {
		*out_ov = true;
		return hi;
	}
	return static_cast<T>(a - b);
}

RegFile::RegFile() {
	memset(regs, 0, sizeof(regs));
}

RegFile::RegFile(const RegFile &other) {
	memcpy(regs, other.regs, sizeof(regs));
}

void RegFile::write(uint32_t index, int32_t value) {
	assert(index <= x31);
	regs[index] = value;
}
void RegFile::uwrite(uint32_t index, uint32_t value) {
	assert(index <= x31);
	regs[index] = (int32_t)value;
}
void RegFile::write8x4(uint32_t index, std::array<int32_t, 4> value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	regs[index] = (value[3] & 0xFF) << 24 | (value[2] & 0xFF) << 16 | (value[1] & 0xFF) << 8 | (value[0] & 0xFF) << 0;
}
void RegFile::uwrite8x4(uint32_t index, std::array<uint32_t, 4> value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	regs[index] = (value[3] & 0xFF) << 24 | (value[2] & 0xFF) << 16 | (value[1] & 0xFF) << 8 | (value[0] & 0xFF) << 0;
}
void RegFile::write16x2(uint32_t index, std::array<int32_t, 2> value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	regs[index] = (value[1] & 0xFFFF) << 16 | (value[0] & 0xFFFF) << 0;
}
void RegFile::uwrite16x2(uint32_t index, std::array<uint32_t, 2> value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	regs[index] = (value[1] & 0xFFFF) << 16 | (value[0] & 0xFFFF) << 0;
}
void RegFile::write32x2(uint32_t index, std::array<int32_t, 2> value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");

	regs[index & ~1] = value[0];
	regs[index | 1] = value[1];
}
void RegFile::uwrite32x2(uint32_t index, std::array<uint32_t, 2> value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");

	regs[index & ~1] = value[0];
	regs[index | 1] = value[1];
}

void RegFile::write64x1(uint32_t index, int64_t value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return write32x2(index, {
	                            (int32_t)(value & 0xFFFF'FFFF),
	                            (int32_t)((value >> 32) & 0xFFFF'FFFF),
	                        });
}
void RegFile::uwrite64x1(uint32_t index, uint64_t value) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return uwrite32x2(index, {
	                             (uint32_t)(value & 0xFFFF'FFFF),
	                             (uint32_t)((value >> 32) & 0xFFFF'FFFF),
	                         });
}

int32_t RegFile::read(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return regs[index];
}

uint32_t RegFile::uread(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return regs[index];
}

std::array<int32_t, 4> RegFile::read8x4(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");

	const auto f = std::array<int8_t, 4>{
	    (int8_t)((regs[index] >> 0) & 0xFF),
	    (int8_t)((regs[index] >> 8) & 0xFF),
	    (int8_t)((regs[index] >> 16) & 0xFF),
	    (int8_t)((regs[index] >> 24) & 0xFF),
	};
	return {f[0], f[1], f[2], f[3]};
}

std::array<uint32_t, 4> RegFile::uread8x4(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");

	const auto f = std::array<uint8_t, 4>{
	    (uint8_t)((regs[index] >> 0) & 0xFF),
	    (uint8_t)((regs[index] >> 8) & 0xFF),
	    (uint8_t)((regs[index] >> 16) & 0xFF),
	    (uint8_t)((regs[index] >> 24) & 0xFF),
	};
	return {f[0], f[1], f[2], f[3]};
}

std::array<int32_t, 2> RegFile::read16x2(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");

	const auto f = std::array<int16_t, 2>{
	    (int16_t)((regs[index] >> 0) & 0xFFFF),
	    (int16_t)((regs[index] >> 16) & 0xFFFF),
	};
	return {f[0], f[1]};
}

std::array<uint32_t, 2> RegFile::uread16x2(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");

	const auto f = std::array<uint16_t, 2>{
	    (uint16_t)((regs[index] >> 0) & 0xFFFF),
	    (uint16_t)((regs[index] >> 16) & 0xFFFF),
	};
	return {f[0], f[1]};
}

std::array<int32_t, 2> RegFile::read32x2(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return {
	    regs[index & ~1],
	    regs[index | 1],
	};
}

std::array<uint32_t, 2> RegFile::uread32x2(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return {
	    (uint32_t)regs[index & ~1],
	    (uint32_t)regs[index | 1],
	};
}

int64_t RegFile::read64x1(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return read32x2(index)[0] | (int64_t)read32x2(index)[1] << 32;
}

uint64_t RegFile::uread64x1(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return uread32x2(index)[0] | (uint64_t)uread32x2(index)[1] << 32;
}

uint32_t RegFile::shamt(uint32_t index) {
	assert(index <= x31);
	return BIT_RANGE(regs[index], 4, 0);
}

int32_t &RegFile::operator[](const uint32_t idx) {
	return regs[idx];
}

#if defined(COLOR_THEME_LIGHT) || defined(COLOR_THEME_DARK)
#define COLORFRMT "\e[38;5;%um%s\e[39m"
#define COLORPRINT(fmt, data) fmt, data
#else
#define COLORFRMT "%s"
#define COLORPRINT(fmt, data) data
#endif

void RegFile::show() {
	for (unsigned i = 0; i < NUM_REGS; ++i) {
		printf(COLORFRMT " = %8x\n", COLORPRINT(regcolors[i], regnames[i]), regs[i]);
	}
}

ISS::ISS(uint32_t hart_id, bool use_E_base_isa) : systemc_name("Core-" + std::to_string(hart_id)) {
	csrs.mhartid.reg = hart_id;
	if (use_E_base_isa)
		csrs.misa.select_E_base_isa();

	sc_core::sc_time qt = tlm::tlm_global_quantum::instance().get();
	cycle_time = sc_core::sc_time(10, sc_core::SC_NS);

	assert(qt >= cycle_time);
	assert(qt % cycle_time == sc_core::SC_ZERO_TIME);

	for (int i = 0; i < Opcode::NUMBER_OF_INSTRUCTIONS; ++i) instr_cycles[i] = cycle_time;

	const sc_core::sc_time memory_access_cycles = 4 * cycle_time;
	const sc_core::sc_time mul_div_cycles = 8 * cycle_time;

	instr_cycles[Opcode::LB] = memory_access_cycles;
	instr_cycles[Opcode::LBU] = memory_access_cycles;
	instr_cycles[Opcode::LH] = memory_access_cycles;
	instr_cycles[Opcode::LHU] = memory_access_cycles;
	instr_cycles[Opcode::LW] = memory_access_cycles;
	instr_cycles[Opcode::SB] = memory_access_cycles;
	instr_cycles[Opcode::SH] = memory_access_cycles;
	instr_cycles[Opcode::SW] = memory_access_cycles;
	instr_cycles[Opcode::MUL] = mul_div_cycles;
	instr_cycles[Opcode::MULH] = mul_div_cycles;
	instr_cycles[Opcode::MULHU] = mul_div_cycles;
	instr_cycles[Opcode::MULHSU] = mul_div_cycles;
	instr_cycles[Opcode::DIV] = mul_div_cycles;
	instr_cycles[Opcode::DIVU] = mul_div_cycles;
	instr_cycles[Opcode::REM] = mul_div_cycles;
	instr_cycles[Opcode::REMU] = mul_div_cycles;
	op = Opcode::UNDEF;
}

void ISS::exec_step() {
	assert(((pc & ~pc_alignment_mask()) == 0) && "misaligned instruction");

	try {
		uint32_t mem_word = instr_mem->load_instr(pc);
		instr = Instruction(mem_word);
	} catch (SimulationTrap &e) {
		op = Opcode::UNDEF;
		instr = Instruction(0);
		throw;
	}

	if (instr.is_compressed()) {
		op = instr.decode_and_expand_compressed(RV32);
		pc += 2;
		if (op != Opcode::UNDEF)
			REQUIRE_ISA(C_ISA_EXT);
	} else {
		op = instr.decode_normal(RV32);
		pc += 4;
	}

	if (trace) {
		printf("core %2u: prv %1x: pc %8x: %s ", csrs.mhartid.reg, prv, last_pc, Opcode::mappingStr[op]);
		switch (Opcode::getType(op)) {
			case Opcode::Type::R:
				printf(COLORFRMT ", " COLORFRMT ", " COLORFRMT, COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
				       COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
				       COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]));
				break;
			case Opcode::Type::I:
				printf(COLORFRMT ", " COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
				       COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]), instr.I_imm());
				break;
			case Opcode::Type::S:
				printf(COLORFRMT ", " COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
				       COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]), instr.S_imm());
				break;
			case Opcode::Type::B:
				printf(COLORFRMT ", " COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
				       COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]), instr.B_imm());
				break;
			case Opcode::Type::U:
				printf(COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]), instr.U_imm());
				break;
			case Opcode::Type::J:
				printf(COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]), instr.J_imm());
				break;
			default:;
		}
		puts("");
	}

	switch (op) {
		case Opcode::UNDEF:
			if (trace)
				std::cout << "[ISS] WARNING: unknown instruction '" << std::to_string(instr.data()) << "' at address '"
				          << std::to_string(last_pc) << "'" << std::endl;
			raise_trap(EXC_ILLEGAL_INSTR, instr.data());
			break;

		case Opcode::ADDI:
			regs[instr.rd()] = regs[instr.rs1()] + instr.I_imm();
			break;

		case Opcode::SLTI:
			regs[instr.rd()] = regs[instr.rs1()] < instr.I_imm();
			break;

		case Opcode::SLTIU:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) < ((uint32_t)instr.I_imm());
			break;

		case Opcode::XORI:
			regs[instr.rd()] = regs[instr.rs1()] ^ instr.I_imm();
			break;

		case Opcode::ORI:
			regs[instr.rd()] = regs[instr.rs1()] | instr.I_imm();
			break;

		case Opcode::ANDI:
			regs[instr.rd()] = regs[instr.rs1()] & instr.I_imm();
			break;

		case Opcode::ADD:
			regs[instr.rd()] = regs[instr.rs1()] + regs[instr.rs2()];
			break;

		case Opcode::SUB:
			regs[instr.rd()] = regs[instr.rs1()] - regs[instr.rs2()];
			break;

		case Opcode::SLL:
			regs[instr.rd()] = regs[instr.rs1()] << regs.shamt(instr.rs2());
			break;

		case Opcode::SLT:
			regs[instr.rd()] = regs[instr.rs1()] < regs[instr.rs2()];
			break;

		case Opcode::SLTU:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) < ((uint32_t)regs[instr.rs2()]);
			break;

		case Opcode::SRL:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) >> regs.shamt(instr.rs2());
			break;

		case Opcode::SRA:
			regs[instr.rd()] = regs[instr.rs1()] >> regs.shamt(instr.rs2());
			break;

		case Opcode::XOR:
			regs[instr.rd()] = regs[instr.rs1()] ^ regs[instr.rs2()];
			break;

		case Opcode::OR:
			regs[instr.rd()] = regs[instr.rs1()] | regs[instr.rs2()];
			break;

		case Opcode::AND:
			regs[instr.rd()] = regs[instr.rs1()] & regs[instr.rs2()];
			break;

		case Opcode::SLLI:
			regs[instr.rd()] = regs[instr.rs1()] << instr.shamt();
			break;

		case Opcode::SRLI:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) >> instr.shamt();
			break;

		case Opcode::SRAI:
			regs[instr.rd()] = regs[instr.rs1()] >> instr.shamt();
			break;

		case Opcode::LUI:
			regs[instr.rd()] = instr.U_imm();
			break;

		case Opcode::AUIPC:
			regs[instr.rd()] = last_pc + instr.U_imm();
			break;

		case Opcode::JAL: {
			auto link = pc;
			pc = last_pc + instr.J_imm();
			trap_check_pc_alignment();
			regs[instr.rd()] = link;
		} break;

		case Opcode::JALR: {
			auto link = pc;
			pc = (regs[instr.rs1()] + instr.I_imm()) & ~1;
			trap_check_pc_alignment();
			regs[instr.rd()] = link;
		} break;

		case Opcode::SB: {
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			mem->store_byte(addr, regs[instr.rs2()]);
		} break;

		case Opcode::SH: {
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			trap_check_addr_alignment<2, false>(addr);
			mem->store_half(addr, regs[instr.rs2()]);
		} break;

		case Opcode::SW: {
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			trap_check_addr_alignment<4, false>(addr);
			mem->store_word(addr, regs[instr.rs2()]);
		} break;

		case Opcode::LB: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			regs[instr.rd()] = mem->load_byte(addr);
		} break;

		case Opcode::LH: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<2, true>(addr);
			regs[instr.rd()] = mem->load_half(addr);
		} break;

		case Opcode::LW: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<4, true>(addr);
			regs[instr.rd()] = mem->load_word(addr);
		} break;

		case Opcode::LBU: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			regs[instr.rd()] = mem->load_ubyte(addr);
		} break;

		case Opcode::LHU: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<2, true>(addr);
			regs[instr.rd()] = mem->load_uhalf(addr);
		} break;

		case Opcode::BEQ:
			if (regs[instr.rs1()] == regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BNE:
			if (regs[instr.rs1()] != regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BLT:
			if (regs[instr.rs1()] < regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BGE:
			if (regs[instr.rs1()] >= regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BLTU:
			if ((uint32_t)regs[instr.rs1()] < (uint32_t)regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BGEU:
			if ((uint32_t)regs[instr.rs1()] >= (uint32_t)regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::FENCE:
		case Opcode::FENCE_I: {
			// not using out of order execution so can be ignored
		} break;

		case Opcode::ECALL: {
			if (sys) {
				sys->execute_syscall(this);
			} else {
				switch (prv) {
					case MachineMode:
						raise_trap(EXC_ECALL_M_MODE, last_pc);
						break;
					case SupervisorMode:
						raise_trap(EXC_ECALL_S_MODE, last_pc);
						break;
					case UserMode:
						raise_trap(EXC_ECALL_U_MODE, last_pc);
						break;
					default:
						throw std::runtime_error("unknown privilege level " + std::to_string(prv));
				}
			}
		} break;

		case Opcode::EBREAK: {
			// TODO: also raise trap and let the SW deal with it?
			status = CoreExecStatus::HitBreakpoint;
		} break;

		case Opcode::CSRRW: {
			auto addr = instr.csr();
			if (is_invalid_csr_access(addr, true)) {
				RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				auto rs1_val = regs[instr.rs1()];
				if (rd != RegFile::zero) {
					regs[instr.rd()] = get_csr_value(addr);
				}
				set_csr_value(addr, rs1_val);
			}
		} break;

		case Opcode::CSRRS: {
			auto addr = instr.csr();
			auto rs1 = instr.rs1();
			auto write = rs1 != RegFile::zero;
			if (is_invalid_csr_access(addr, write)) {
				RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				auto rs1_val = regs[rs1];
				auto csr_val = get_csr_value(addr);
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val | rs1_val);
			}
		} break;

		case Opcode::CSRRC: {
			auto addr = instr.csr();
			auto rs1 = instr.rs1();
			auto write = rs1 != RegFile::zero;
			if (is_invalid_csr_access(addr, write)) {
				RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				auto rs1_val = regs[rs1];
				auto csr_val = get_csr_value(addr);
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val & ~rs1_val);
			}
		} break;

		case Opcode::CSRRWI: {
			auto addr = instr.csr();
			if (is_invalid_csr_access(addr, true)) {
				RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				if (rd != RegFile::zero) {
					regs[rd] = get_csr_value(addr);
				}
				set_csr_value(addr, instr.zimm());
			}
		} break;

		case Opcode::CSRRSI: {
			auto addr = instr.csr();
			auto zimm = instr.zimm();
			auto write = zimm != 0;
			if (is_invalid_csr_access(addr, write)) {
				RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto csr_val = get_csr_value(addr);
				auto rd = instr.rd();
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val | zimm);
			}
		} break;

		case Opcode::CSRRCI: {
			auto addr = instr.csr();
			auto zimm = instr.zimm();
			auto write = zimm != 0;
			if (is_invalid_csr_access(addr, write)) {
				RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto csr_val = get_csr_value(addr);
				auto rd = instr.rd();
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val & ~zimm);
			}
		} break;

		case Opcode::MUL: {
			REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = (int64_t)regs[instr.rs1()] * (int64_t)regs[instr.rs2()];
			regs[instr.rd()] = ans & 0xFFFFFFFF;
		} break;

		case Opcode::MULH: {
			REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = (int64_t)regs[instr.rs1()] * (int64_t)regs[instr.rs2()];
			regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
		} break;

		case Opcode::MULHU: {
			REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = ((uint64_t)(uint32_t)regs[instr.rs1()]) * (uint64_t)((uint32_t)regs[instr.rs2()]);
			regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
		} break;

		case Opcode::MULHSU: {
			REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = (int64_t)regs[instr.rs1()] * (uint64_t)((uint32_t)regs[instr.rs2()]);
			regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
		} break;

		case Opcode::DIV: {
			REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = -1;
			} else if (a == REG_MIN && b == -1) {
				regs[instr.rd()] = a;
			} else {
				regs[instr.rd()] = a / b;
			}
		} break;

		case Opcode::DIVU: {
			REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = -1;
			} else {
				regs[instr.rd()] = (uint32_t)a / (uint32_t)b;
			}
		} break;

		case Opcode::REM: {
			REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = a;
			} else if (a == REG_MIN && b == -1) {
				regs[instr.rd()] = 0;
			} else {
				regs[instr.rd()] = a % b;
			}
		} break;

		case Opcode::REMU: {
			REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = a;
			} else {
				regs[instr.rd()] = (uint32_t)a % (uint32_t)b;
			}
		} break;

		case Opcode::LR_W: {
			REQUIRE_ISA(A_ISA_EXT);
			uint32_t addr = regs[instr.rs1()];
			trap_check_addr_alignment<4, true>(addr);
			regs[instr.rd()] = mem->atomic_load_reserved_word(addr);
			if (lr_sc_counter == 0)
				lr_sc_counter = 17;  // this instruction + 16 additional ones, (an over-approximation) to cover the
				                     // RISC-V forward progress property
		} break;

		case Opcode::SC_W: {
			REQUIRE_ISA(A_ISA_EXT);
			uint32_t addr = regs[instr.rs1()];
			trap_check_addr_alignment<4, false>(addr);
			uint32_t val = regs[instr.rs2()];
			regs[instr.rd()] = 1;  // failure by default (in case a trap is thrown)
			regs[instr.rd()] =
			    mem->atomic_store_conditional_word(addr, val) ? 0 : 1;  // overwrite result (in case no trap is thrown)
			lr_sc_counter = 0;
		} break;

		case Opcode::AMOSWAP_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) {
				(void)a;
				return b;
			});
		} break;

		case Opcode::AMOADD_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a + b; });
		} break;

		case Opcode::AMOXOR_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a ^ b; });
		} break;

		case Opcode::AMOAND_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a & b; });
		} break;

		case Opcode::AMOOR_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a | b; });
		} break;

		case Opcode::AMOMIN_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::min(a, b); });
		} break;

		case Opcode::AMOMINU_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::min((uint32_t)a, (uint32_t)b); });
		} break;

		case Opcode::AMOMAX_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::max(a, b); });
		} break;

		case Opcode::AMOMAXU_W: {
			REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::max((uint32_t)a, (uint32_t)b); });
		} break;

			// RV32F Extension

		case Opcode::FLW: {
			REQUIRE_ISA(F_ISA_EXT);
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<4, true>(addr);
			fp_regs.write(RD, float32_t{(uint32_t)mem->load_word(addr)});
		} break;

		case Opcode::FSW: {
			REQUIRE_ISA(F_ISA_EXT);
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			trap_check_addr_alignment<4, false>(addr);
			mem->store_word(addr, fp_regs.u32(RS2));
		} break;

		case Opcode::FADD_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_add(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FSUB_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_sub(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FMUL_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mul(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FDIV_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_div(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FSQRT_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_sqrt(fp_regs.f32(RS1)));
			fp_finish_instr();
		} break;

		case Opcode::FMIN_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();

			bool rs1_smaller = f32_lt_quiet(fp_regs.f32(RS1), fp_regs.f32(RS2)) ||
			                   (f32_eq(fp_regs.f32(RS1), fp_regs.f32(RS2)) && f32_isNegative(fp_regs.f32(RS1)));

			if (f32_isNaN(fp_regs.f32(RS1)) && f32_isNaN(fp_regs.f32(RS2))) {
				fp_regs.write(RD, f32_defaultNaN);
			} else {
				if (rs1_smaller)
					fp_regs.write(RD, fp_regs.f32(RS1));
				else
					fp_regs.write(RD, fp_regs.f32(RS2));
			}

			fp_finish_instr();
		} break;

		case Opcode::FMAX_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();

			bool rs1_greater = f32_lt_quiet(fp_regs.f32(RS2), fp_regs.f32(RS1)) ||
			                   (f32_eq(fp_regs.f32(RS2), fp_regs.f32(RS1)) && f32_isNegative(fp_regs.f32(RS2)));

			if (f32_isNaN(fp_regs.f32(RS1)) && f32_isNaN(fp_regs.f32(RS2))) {
				fp_regs.write(RD, f32_defaultNaN);
			} else {
				if (rs1_greater)
					fp_regs.write(RD, fp_regs.f32(RS1));
				else
					fp_regs.write(RD, fp_regs.f32(RS2));
			}

			fp_finish_instr();
		} break;

		case Opcode::FMADD_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(fp_regs.f32(RS1), fp_regs.f32(RS2), fp_regs.f32(RS3)));
			fp_finish_instr();
		} break;

		case Opcode::FMSUB_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(fp_regs.f32(RS1), fp_regs.f32(RS2), f32_neg(fp_regs.f32(RS3))));
			fp_finish_instr();
		} break;

		case Opcode::FNMADD_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(f32_neg(fp_regs.f32(RS1)), fp_regs.f32(RS2), f32_neg(fp_regs.f32(RS3))));
			fp_finish_instr();
		} break;

		case Opcode::FNMSUB_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(f32_neg(fp_regs.f32(RS1)), fp_regs.f32(RS2), fp_regs.f32(RS3)));
			fp_finish_instr();
		} break;

		case Opcode::FCVT_W_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			regs[RD] = f32_to_i32(fp_regs.f32(RS1), softfloat_roundingMode, true);
			fp_finish_instr();
		} break;

		case Opcode::FCVT_WU_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			regs[RD] = f32_to_ui32(fp_regs.f32(RS1), softfloat_roundingMode, true);
			fp_finish_instr();
		} break;

		case Opcode::FCVT_S_W: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, i32_to_f32(regs[RS1]));
			fp_finish_instr();
		} break;

		case Opcode::FCVT_S_WU: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, ui32_to_f32(regs[RS1]));
			fp_finish_instr();
		} break;

		case Opcode::FSGNJ_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f32(RS1);
			auto f2 = fp_regs.f32(RS2);
			fp_regs.write(RD, float32_t{(f1.v & ~F32_SIGN_BIT) | (f2.v & F32_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FSGNJN_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f32(RS1);
			auto f2 = fp_regs.f32(RS2);
			fp_regs.write(RD, float32_t{(f1.v & ~F32_SIGN_BIT) | (~f2.v & F32_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FSGNJX_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f32(RS1);
			auto f2 = fp_regs.f32(RS2);
			fp_regs.write(RD, float32_t{f1.v ^ (f2.v & F32_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FMV_W_X: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_regs.write(RD, float32_t{(uint32_t)regs[RS1]});
			fp_set_dirty();
		} break;

		case Opcode::FMV_X_W: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = fp_regs.u32(RS1);
		} break;

		case Opcode::FEQ_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_eq(fp_regs.f32(RS1), fp_regs.f32(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FLT_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_lt(fp_regs.f32(RS1), fp_regs.f32(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FLE_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_le(fp_regs.f32(RS1), fp_regs.f32(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FCLASS_S: {
			REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_classify(fp_regs.f32(RS1));
		} break;

			// RV32D Extension

		case Opcode::FLD: {
			REQUIRE_ISA(D_ISA_EXT);
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<8, true>(addr);
			fp_regs.write(RD, float64_t{(uint64_t)mem->load_double(addr)});
		} break;

		case Opcode::FSD: {
			REQUIRE_ISA(D_ISA_EXT);
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			trap_check_addr_alignment<8, false>(addr);
			mem->store_double(addr, fp_regs.f64(RS2).v);
		} break;

		case Opcode::FADD_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_add(fp_regs.f64(RS1), fp_regs.f64(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FSUB_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_sub(fp_regs.f64(RS1), fp_regs.f64(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FMUL_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_mul(fp_regs.f64(RS1), fp_regs.f64(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FDIV_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_div(fp_regs.f64(RS1), fp_regs.f64(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FSQRT_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_sqrt(fp_regs.f64(RS1)));
			fp_finish_instr();
		} break;

		case Opcode::FMIN_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();

			bool rs1_smaller = f64_lt_quiet(fp_regs.f64(RS1), fp_regs.f64(RS2)) ||
			                   (f64_eq(fp_regs.f64(RS1), fp_regs.f64(RS2)) && f64_isNegative(fp_regs.f64(RS1)));

			if (f64_isNaN(fp_regs.f64(RS1)) && f64_isNaN(fp_regs.f64(RS2))) {
				fp_regs.write(RD, f64_defaultNaN);
			} else {
				if (rs1_smaller)
					fp_regs.write(RD, fp_regs.f64(RS1));
				else
					fp_regs.write(RD, fp_regs.f64(RS2));
			}

			fp_finish_instr();
		} break;

		case Opcode::FMAX_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();

			bool rs1_greater = f64_lt_quiet(fp_regs.f64(RS2), fp_regs.f64(RS1)) ||
			                   (f64_eq(fp_regs.f64(RS2), fp_regs.f64(RS1)) && f64_isNegative(fp_regs.f64(RS2)));

			if (f64_isNaN(fp_regs.f64(RS1)) && f64_isNaN(fp_regs.f64(RS2))) {
				fp_regs.write(RD, f64_defaultNaN);
			} else {
				if (rs1_greater)
					fp_regs.write(RD, fp_regs.f64(RS1));
				else
					fp_regs.write(RD, fp_regs.f64(RS2));
			}

			fp_finish_instr();
		} break;

		case Opcode::FMADD_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_mulAdd(fp_regs.f64(RS1), fp_regs.f64(RS2), fp_regs.f64(RS3)));
			fp_finish_instr();
		} break;

		case Opcode::FMSUB_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_mulAdd(fp_regs.f64(RS1), fp_regs.f64(RS2), f64_neg(fp_regs.f64(RS3))));
			fp_finish_instr();
		} break;

		case Opcode::FNMADD_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_mulAdd(f64_neg(fp_regs.f64(RS1)), fp_regs.f64(RS2), f64_neg(fp_regs.f64(RS3))));
			fp_finish_instr();
		} break;

		case Opcode::FNMSUB_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_mulAdd(f64_neg(fp_regs.f64(RS1)), fp_regs.f64(RS2), fp_regs.f64(RS3)));
			fp_finish_instr();
		} break;

		case Opcode::FSGNJ_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f64(RS1);
			auto f2 = fp_regs.f64(RS2);
			fp_regs.write(RD, float64_t{(f1.v & ~F64_SIGN_BIT) | (f2.v & F64_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FSGNJN_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f64(RS1);
			auto f2 = fp_regs.f64(RS2);
			fp_regs.write(RD, float64_t{(f1.v & ~F64_SIGN_BIT) | (~f2.v & F64_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FSGNJX_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f64(RS1);
			auto f2 = fp_regs.f64(RS2);
			fp_regs.write(RD, float64_t{f1.v ^ (f2.v & F64_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FCVT_S_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f64_to_f32(fp_regs.f64(RS1)));
			fp_finish_instr();
		} break;

		case Opcode::FCVT_D_S: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_to_f64(fp_regs.f32(RS1)));
			fp_finish_instr();
		} break;

		case Opcode::FEQ_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f64_eq(fp_regs.f64(RS1), fp_regs.f64(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FLT_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f64_lt(fp_regs.f64(RS1), fp_regs.f64(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FLE_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f64_le(fp_regs.f64(RS1), fp_regs.f64(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FCLASS_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = (int64_t)f64_classify(fp_regs.f64(RS1));
		} break;

		case Opcode::FCVT_W_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			regs[RD] = f64_to_i32(fp_regs.f64(RS1), softfloat_roundingMode, true);
			fp_finish_instr();
		} break;

		case Opcode::FCVT_WU_D: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			regs[RD] = (int32_t)f64_to_ui32(fp_regs.f64(RS1), softfloat_roundingMode, true);
			fp_finish_instr();
		} break;

		case Opcode::FCVT_D_W: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, i32_to_f64((int32_t)regs[RS1]));
			fp_finish_instr();
		} break;

		case Opcode::FCVT_D_WU: {
			REQUIRE_ISA(D_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, ui32_to_f64((int32_t)regs[RS1]));
			fp_finish_instr();
		} break;

			// privileged instructions

		case Opcode::WFI:
			// NOTE: only a hint, can be implemented as NOP
			// std::cout << "[sim:wfi] CSR mstatus.mie " << csrs.mstatus->mie << std::endl;
			release_lr_sc_reservation();

			if (s_mode() && csrs.mstatus.fields.tw)
				raise_trap(EXC_ILLEGAL_INSTR, instr.data());

			if (u_mode() && csrs.misa.has_supervisor_mode_extension())
				raise_trap(EXC_ILLEGAL_INSTR, instr.data());

			if (!ignore_wfi && !has_local_pending_enabled_interrupts())
				sc_core::wait(wfi_event);
			break;

		case Opcode::SFENCE_VMA:
			if (s_mode() && csrs.mstatus.fields.tvm)
				raise_trap(EXC_ILLEGAL_INSTR, instr.data());
			mem->flush_tlb();
			break;

		case Opcode::URET:
			if (!csrs.misa.has_user_mode_extension())
				raise_trap(EXC_ILLEGAL_INSTR, instr.data());
			return_from_trap_handler(UserMode);
			break;

		case Opcode::SRET:
			if (!csrs.misa.has_supervisor_mode_extension() || (s_mode() && csrs.mstatus.fields.tsr))
				raise_trap(EXC_ILLEGAL_INSTR, instr.data());
			return_from_trap_handler(SupervisorMode);
			break;

		case Opcode::MRET:
			return_from_trap_handler(MachineMode);
			break;

		// *** P Extension ***
		// SIMD Add 8-bit
		case Opcode::ADD8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			regs.write8x4(instr.rd(), {
			                              rs1[0] + rs2[0],
			                              rs1[1] + rs2[1],
			                              rs1[2] + rs2[2],
			                              rs1[3] + rs2[3],
			                          });
		} break;

		// SIMD Add 16-bit
		case Opcode::ADD16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {
			                               rs1[0] + rs2[0],
			                               rs1[1] + rs2[1],
			                           });
		} break;

		// SIMD Add 64-bit
		case Opcode::ADD64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read64x1(instr.rs1());
			const auto rs2 = regs.read64x1(instr.rs2());
			regs.write64x1(instr.rd(), rs1 + rs2);
		} break;

		// Average with rounding
		case Opcode::AVE: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			regs.write(instr.rd(), (int32_t)(((int64_t)rs1 + rs2 + 1) >> 1));
		} break;

		// Bit Reverse
		case Opcode::BITREV: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint32_t)regs.read(instr.rs1());
			const auto rs2 = (uint32_t)regs.read(instr.rs2());
			auto n = rs1;
			n = (n & 0xffff0000) >> 16 | (n & 0x0000ffff) << 16;
			n = (n & 0xff00ff00) >> 8 | (n & 0x00ff00ff) << 8;
			n = (n & 0xf0f0f0f0) >> 4 | (n & 0x0f0f0f0f) << 4;
			n = (n & 0xcccccccc) >> 2 | (n & 0x33333333) << 2;
			n = (n & 0xaaaaaaaa) >> 1 | (n & 0x55555555) << 1;
			n = n >> (32 - rs2);
			regs.write(instr.rd(), (int32_t)n);
		} break;

		// Bit Reverse Immediate
		case Opcode::BITREVI: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint32_t)regs.read(instr.rs1());
			const auto rs2 = (uint32_t)instr.rs2();
			auto n = rs1;
			n = (n & 0xffff0000) >> 16 | (n & 0x0000ffff) << 16;
			n = (n & 0xff00ff00) >> 8 | (n & 0x00ff00ff) << 8;
			n = (n & 0xf0f0f0f0) >> 4 | (n & 0x0f0f0f0f) << 4;
			n = (n & 0xcccccccc) >> 2 | (n & 0x33333333) << 2;
			n = (n & 0xaaaaaaaa) >> 1 | (n & 0x55555555) << 1;
			n = n >> (32 - rs2);
			regs.write(instr.rd(), (int32_t)n);
		} break;

		// SIMD 8-bit Count Leading Redundant Sign
		case Opcode::CLRS8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			std::array<int32_t, 4> cnt = {};
			const std::array<int32_t, 4> sign_bit = {
			    (rs1[0] >> 7) & 1,
			    (rs1[1] >> 7) & 1,
			    (rs1[2] >> 7) & 1,
			    (rs1[3] >> 7) & 1,
			};

			for (int32_t lane = 0; lane < 4; ++lane) {
				for (int32_t i = 6; i >= 0; --i) {
					if (cnt[lane] + (((rs1[lane] >> i) & 1) == sign_bit[lane]))
						++cnt[lane];
					else
						break;
				}
			}

			regs.write8x4(instr.rd(), cnt);
		} break;

		// SIMD 16-bit Count Leading Redundant Sign
		case Opcode::CLRS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			std::array<int32_t, 2> cnt = {};
			const std::array<int32_t, 2> sign_bit = {
			    (rs1[0] >> 15) & 1,
			    (rs1[1] >> 15) & 1,
			};

			for (int32_t lane = 0; lane < 2; ++lane) {
				for (int32_t i = 14; i >= 0; --i) {
					if (cnt[lane] + (((rs1[lane] >> i) & 1) == sign_bit[lane]))
						++cnt[lane];
					else
						break;
				}
			}
			regs.write16x2(instr.rd(), cnt);
		} break;

		// SIMD 32-bit Count Leading Redundant Sign
		case Opcode::CLRS32: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			int32_t cnt = {};
			int32_t sign_bit = (rs1 >> 31) & 1;

			for (int32_t i = 30; i >= 0; --i) {
				if (cnt + (((rs1 >> i) & 1) == sign_bit))
					++cnt;
				else
					break;
			}

			regs.write(instr.rd(), cnt);
		} break;

		// SIMD 8-bit Count Leading Zero
		case Opcode::CLZ8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			std::array<int32_t, 4> cnt = {};

			for (int32_t lane = 0; lane < 4; ++lane) {
				for (int32_t i = 0; i < 8; ++i) {
					if (cnt[lane] + (((rs1[lane] >> i) & 1) == 0))
						++cnt[lane];
					else
						break;
				}
			}
			regs.write8x4(instr.rd(), cnt);
		} break;

		// SIMD 16-bit Count Leading Zero
		case Opcode::CLZ16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			std::array<int32_t, 2> cnt = {};

			for (int32_t lane = 0; lane < 2; ++lane) {
				for (int32_t i = 0; i < 16; ++i) {
					if (cnt[lane] + (((rs1[lane] >> i) & 1) == 0))
						++cnt[lane];
					else
						break;
				}
			}
			regs.write16x2(instr.rd(), cnt);
		} break;

		// SIMD 32-bit Count Leading Zero
		case Opcode::CLZ32: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			int32_t cnt = {};

			for (int32_t i = 0; i < 32; ++i) {
				if (cnt + (((rs1 >> i) & 1) == 0))
					++cnt;
				else
					break;
			}

			regs.write(instr.rd(), cnt);
		} break;

		// SIMD 8-bit Integer Compare Equal
		case Opcode::CMPEQ8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			std::array<int32_t, 4> rd = {};

			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] == rs2[lane];
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Integer Compare Equal
		case Opcode::CMPEQ16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};

			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] == rs2[lane] ? 0xFFFF'FFFF : 0;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Cross Addition & Subtraction
		case Opcode::CRAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {rs1[0] - rs2[1], rs1[1] + rs2[0]};
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Cross Subtraction & Addition
		case Opcode::CRSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {rs1[0] + rs2[1], rs1[1] - rs2[0]};
			regs.write16x2(instr.rd(), rd);
		} break;

		// Insert Byte
		case Opcode::INSB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			auto rd = regs.read8x4(instr.rd());
			const auto imm = BIT_SLICE(instr.data(), 21, 20);
			rd[imm] = rs1[0];
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-Bit Saturating Absolute
		case Opcode::KABS8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			std::array<int32_t, 4> rd = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 2; ++lane) {
				const auto sat = rs1[lane] == INT8_MIN;
				rd[lane] = sat ? INT8_MAX : abs(rs1[lane]);
				ov |= sat;
			}
			regs.write8x4(instr.rd(), rd);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-Bit Saturating Absolute
		case Opcode::KABS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			std::array<int32_t, 2> rd = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 2; ++lane) {
				ov |= rs1[lane] == INT16_MIN;
				rd[lane] = ov ? INT16_MAX : abs(rs1[lane]);
			}
			regs.write16x2(instr.rd(), rd);
			csrs.vxsat.reg |= ov;
		} break;

		// Scalar 32-bit Absolute Value with Saturation
		case Opcode::KABSW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			int32_t rd = {};
			const auto ov = rs1 == INT32_MIN;
			rd = ov ? INT32_MAX : abs(rs1);
			regs.write(instr.rd(), rd);
			csrs.vxsat.reg |= ov;
		} break;

		// Scalar 32-bit Absolute Value with Saturation
		case Opcode::KADD8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			bool ov = false;
			regs.write8x4(instr.rd(), {
			                              sat_add<int8_t>(&ov, rs1[0], rs2[0]),
			                              sat_add<int8_t>(&ov, rs1[1], rs2[1]),
			                              sat_add<int8_t>(&ov, rs1[2], rs2[2]),
			                              sat_add<int8_t>(&ov, rs1[3], rs2[3]),
			                          });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Signed Saturating Addition
		case Opcode::KADD16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write16x2(instr.rd(), {
			                               sat_add<int16_t>(&ov, rs1[0], rs2[0]),
			                               sat_add<int16_t>(&ov, rs1[1], rs2[1]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// 64-bit Signed Saturating Addition
		case Opcode::KADD64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read64x1(instr.rs1());
			const auto rs2 = regs.read64x1(instr.rs2());
			bool ov = false;
			regs.write64x1(instr.rd(), sat_add<int64_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Addition with Q15 Saturation
		case Opcode::KADDH: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write(instr.rd(), sat_add<int16_t>(&ov, rs1[0], rs2[0]));
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Addition with Q31 Saturation
		case Opcode::KADDW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			bool ov = false;
			regs.write(instr.rd(), sat_add<int32_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Signed Saturating Cross Addition & Subtraction
		case Opcode::KCRAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write16x2(instr.rd(), {
			                               sat_sub<int16_t>(&ov, rs1[0], rs2[1]),
			                               sat_add<int16_t>(&ov, rs1[1], rs2[0]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Signed Saturating Cross Subtraction & Addition
		case Opcode::KCRSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write16x2(instr.rd(), {
			                               sat_add<int16_t>(&ov, rs1[0], rs2[1]),
			                               sat_sub<int16_t>(&ov, rs1[1], rs2[0]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Double Multiply B16 x B16
		case Opcode::KDMBB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res = rs1[0] * rs2[0];
			const auto ov = rs1[0] == 0x8000 && rs2[0] == 0x8000;
			regs.write(instr.rd(), ov ? INT32_MAX : res << 1);
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Double Multiply B16 x T16
		case Opcode::KDMBT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res = rs1[0] * rs2[1];
			const auto ov = rs1[0] == 0x8000 && rs2[1] == 0x8000;
			regs.write(instr.rd(), ov ? INT32_MAX : res << 1);
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Double Multiply T16 x T16
		case Opcode::KDMTT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res = rs1[1] * rs2[1];
			const auto ov = rs1[1] == 0x8000 && rs2[1] == 0x8000;
			regs.write(instr.rd(), ov ? INT32_MAX : res << 1);
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Double Multiply Addition B16 x B16
		case Opcode::KDMABB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = rs1[0] * rs2[0];
			auto ov = rs1[0] == 0x8000 && rs2[0] == 0x8000;
			const auto sat_res = ov ? INT32_MAX : res << 1;
			regs.write(instr.rd(), sat_add<int32_t>(&ov, rd, sat_res));
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Double Multiply Addition B16 x T16
		case Opcode::KDMABT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = rs1[0] * rs2[1];
			auto ov = rs1[0] == 0x8000 && rs2[1] == 0x8000;
			const auto sat_res = ov ? INT32_MAX : res << 1;
			regs.write(instr.rd(), sat_add<int32_t>(&ov, rd, sat_res));
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Double Multiply Addition T16 x T16
		case Opcode::KDMATT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = rs1[1] * rs2[1];
			auto ov = rs1[1] == 0x8000 && rs2[1] == 0x8000;
			const auto sat_res = ov ? INT32_MAX : res << 1;
			regs.write(instr.rd(), sat_add<int32_t>(&ov, rd, sat_res));
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Signed Saturating Q7 Multiply
		case Opcode::KHM8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			std::array<bool, 4> ov = {};
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				const auto res = (rs1[lane] * rs2[lane]) >> 7;
				ov[lane] = rs1[lane] == 0x80 && rs2[lane] == 0x80;
				const auto sat_res = ov[lane] ? INT8_MAX : res;
				rd[lane] = sat_res;
			}
			regs.write8x4(instr.rd(), rd);
			csrs.vxsat.reg |= ov[0] | ov[1] | ov[2] | ov[3];
		} break;

		// SIMD Signed Saturating Crossed Q7 Multiply
		case Opcode::KHMX8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			std::array<bool, 4> ov = {};
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				const auto res = (rs1[lane] * rs2[lane % 2 ? lane - 1 : lane + 1]) >> 7;
				ov[lane] = rs1[lane] == 0x80 && rs2[lane % 2 ? lane - 1 : lane + 1] == 0x80;
				const auto sat_res = ov[lane] ? INT8_MAX : res;
				rd[lane] = sat_res;
			}
			regs.write8x4(instr.rd(), rd);
			csrs.vxsat.reg |= ov[0] | ov[1] | ov[2] | ov[3];
		} break;

		// SIMD Signed Saturating Q15 Multiply
		case Opcode::KHM16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<bool, 2> ov = {};
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				const auto res = (rs1[lane] * rs2[lane]) >> 15;
				ov[lane] = rs1[lane] == 0x8000 && rs2[lane] == 0x8000;
				const auto sat_res = ov[lane] ? INT8_MAX : res;
				rd[lane] = sat_res;
			}
			regs.write16x2(instr.rd(), rd);
			csrs.vxsat.reg |= ov[0] | ov[1];
		} break;

		// SIMD Signed Saturating Crossed Q15 Multiply
		case Opcode::KHMX16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<bool, 2> ov = {};
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				const auto res = (rs1[lane] * rs2[lane % 2 ? lane - 1 : lane + 1]) >> 15;
				ov[lane] = rs1[lane] == 0x8000 && rs2[lane % 2 ? lane - 1 : lane + 1] == 0x8000;
				const auto sat_res = ov[lane] ? INT8_MAX : res;
				rd[lane] = sat_res;
			}
			regs.write16x2(instr.rd(), rd);
			csrs.vxsat.reg |= ov[0] | ov[1];
		} break;

		// Signed Saturating Half Multiply B16 x B16
		case Opcode::KHMBB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res = rs1[0] * rs2[0];
			const auto ov = rs1[0] == 0x8000 && rs2[0] == 0x8000;
			std::array<int32_t, 2> rd = {ov ? INT16_MAX : res >> 15, 0};
			regs.write16x2(instr.rd(), rd);
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Half Multiply B16 x T16
		case Opcode::KHMBT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res = rs1[0] * rs2[1];
			const auto ov = rs1[0] == 0x8000 && rs2[1] == 0x8000;
			std::array<int32_t, 2> rd = {ov ? INT16_MAX : res >> 15, 0};
			regs.write16x2(instr.rd(), rd);
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Saturating Half Multiply T16 x T16
		case Opcode::KHMTT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res = rs1[1] * rs2[1];
			const auto ov = rs1[1] == 0x8000 && rs2[1] == 0x8000;
			std::array<int32_t, 2> rd = {ov ? INT16_MAX : res >> 15, 0};
			regs.write16x2(instr.rd(), rd);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Multiply Bottom Halfs & Add
		case Opcode::KMABB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = rs1[0] * rs2[0];
			bool ov = false;
			regs.write(instr.rd(), sat_add<int32_t>(&ov, res, rd));
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Multiply Bottom & Top Halfs & Add
		case Opcode::KMABT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = rs1[0] * rs2[1];
			bool ov = false;
			regs.write(instr.rd(), sat_add<int32_t>(&ov, res, rd));
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Multiply Top Halfs & Add
		case Opcode::KMATT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = rs1[1] * rs2[1];
			bool ov = false;
			regs.write(instr.rd(), sat_add<int32_t>(&ov, res, rd));
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Multiply Two Halfs and Two Adds
		case Opcode::KMADA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res0 = rs1[0] * rs2[0];
			const auto res1 = rs1[1] * rs2[1];
			bool ov = false;
			const auto res_tmp = sat_add<int32_t>(&ov, res0, res1);
			const auto res = sat_add<int32_t>(&ov, res_tmp, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Crossed Multiply Two Halfs and Two Adds
		case Opcode::KMAXDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res0 = rs1[0] * rs2[1];
			const auto res1 = rs1[1] * rs2[0];
			bool ov = false;
			const auto res_tmp = sat_add<int32_t>(&ov, res0, res1);
			const auto res = sat_add<int32_t>(&ov, res_tmp, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Multiply Two Halfs & Subtract & Add
		case Opcode::KMADS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res0 = rs1[0] * rs2[0];
			const auto res1 = rs1[1] * rs2[1];
			bool ov = false;
			const auto res_tmp = sat_sub<int32_t>(&ov, res1, res0);
			const auto res = sat_add<int32_t>(&ov, res_tmp, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Multiply Two Halfs & Reverse Subtract & Add
		case Opcode::KMADRS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res0 = rs1[0] * rs2[0];
			const auto res1 = rs1[1] * rs2[1];
			bool ov = false;
			const auto res_tmp = sat_sub<int32_t>(&ov, res0, res1);
			const auto res = sat_add<int32_t>(&ov, res_tmp, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Crossed Multiply Two Halfs & Subtract & Add
		case Opcode::KMAXDS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res0 = rs1[1] * rs2[0];
			const auto res1 = rs1[0] * rs2[1];
			bool ov = false;
			const auto res_tmp = sat_sub<int32_t>(&ov, res0, res1);
			const auto res = sat_add<int32_t>(&ov, res_tmp, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Multiply and Saturating Add to 64-Bit Data
		case Opcode::KMAR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			const auto res = rs1 * rs2;
			bool ov = false;
			const auto res2 = sat_add<int64_t>(&ov, res, rd);
			regs.write64x1(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Signed Multiply Two Halfs and Add
		case Opcode::KMDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res0 = rs1[0] * rs2[0];
			const auto res1 = rs1[1] * rs2[1];
			bool ov = false;
			const auto res = sat_add<int32_t>(&ov, res0, res1);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Signed Crossed Multiply Two Halfs and Add
		case Opcode::KMXDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto res0 = rs1[0] * rs2[1];
			const auto res1 = rs1[1] * rs2[0];
			bool ov = false;
			const auto res = sat_add<int32_t>(&ov, res0, res1);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Add
		case Opcode::KMMAC: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = ((int64_t)rs1 * rs2) >> 32;
			bool ov = false;
			const auto res2 = sat_add<int32_t>(&ov, res, rd);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Add with Rounding
		case Opcode::KMMAC_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (((rs1 * rs2) >> 31) + 1) >> 1;
			bool ov = false;
			const auto res2 = sat_add<int32_t>(&ov, res, rd);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Bottom Half and Add
		case Opcode::KMMAWB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (rs1 * rs2[0]) >> 16;
			bool ov = false;
			const auto res2 = sat_add<int32_t>(&ov, res, rd);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Bottom Half and Add with Rounding
		case Opcode::KMMAWB_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (((rs1 * rs2[0]) >> 15) + 1) >> 1;
			bool ov = false;
			const auto res2 = sat_add<int32_t>(&ov, res, rd);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Bottom Half & 2 and Add
		case Opcode::KMMAWB2: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto sat_mul = rs1 == 0x8000'0000 || rs2[0] == 0x8000;
			const auto res_mul = sat_mul ? 0x7FFF'FFFF : (rs1 * rs2[0]) >> 15;
			bool ov = false;
			const auto res = sat_add<int32_t>(&ov, res_mul, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Bottom Half & 2 and Add with Rounding
		case Opcode::KMMAWB2_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto sat_mul = rs1 == 0x8000'0000 || rs2[0] == 0x8000;
			const auto res_mul = sat_mul ? 0x7FFF'FFFF : (((rs1 * rs2[0]) >> 14) + 1) >> 1;
			bool ov = false;
			const auto res = sat_add<int32_t>(&ov, res_mul, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Top Half and Add
		case Opcode::KMMAWT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (rs1 * rs2[1]) >> 16;
			bool ov = false;
			const auto res2 = sat_add<int32_t>(&ov, res, rd);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Top Half and Add with Rounding
		case Opcode::KMMAWT_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (((rs1 * rs2[1]) >> 15) + 1) >> 1;
			bool ov = false;
			const auto res2 = sat_add<int32_t>(&ov, res, rd);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Top Half & 2 and Add
		case Opcode::KMMAWT2: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto sat_mul = (uint32_t)rs1 == 0x8000'0000 || rs2[1] == 0x8000;
			const auto res_mul = sat_mul ? 0x7FFF'FFFF : ((int64_t)rs1 * rs2[1]) >> 15;
			bool ov = false;
			const auto res = sat_add<int32_t>(&ov, res_mul, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Top Half & 2 and Add with Rounding
		case Opcode::KMMAWT2_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto sat_mul = rs1 == 0x8000'0000 || rs2[1] == 0x8000;
			const auto res_mul = sat_mul ? 0x7FFF'FFFF : (((rs1 * rs2[1]) >> 14) + 1) >> 1;
			bool ov = false;
			const auto res = sat_add<int32_t>(&ov, res_mul, rd);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Subtract
		case Opcode::KMMSB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (rs1 * rs2) >> 32;
			bool ov = false;
			const auto res2 = sat_sub<int32_t>(&ov, rd, res);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Subtraction with Rounding
		case Opcode::KMMSB_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (((rs1 * rs2) >> 31) + 1) >> 1;
			bool ov = false;
			const auto res2 = sat_sub<int32_t>(&ov, rd, res);
			regs.write(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Bottom Half & 2
		case Opcode::KMMWB2: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto ov = rs1 == 0x8000'0000 || rs2[0] == 0x8000;
			const auto res = ov ? 0x7FFF'FFFF : (rs1 * rs2[0]) >> 15;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Bottom Half & 2 with Rounding
		case Opcode::KMMWB2_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto ov = rs1 == 0x8000'0000 || rs2[0] == 0x8000;
			const auto res = ov ? 0x7FFF'FFFF : (((rs1 * rs2[0]) >> 14) + 1) >> 1;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Top Half & 2
		case Opcode::KMMWT2: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto ov = rs1 == 0x8000'0000 || rs2[1] == 0x8000;
			const auto res = ov ? 0x7FFF'FFFF : (rs1 * rs2[1]) >> 15;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word and Top Half & 2 with Rounding
		case Opcode::KMMWT2_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto ov = rs1 == 0x8000'0000 || rs2[1] == 0x8000;
			const auto res = ov ? 0x7FFF'FFFF : (((rs1 * rs2[1]) >> 14) + 1) >> 1;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Multiply Two Halfs & Add & Subtract
		case Opcode::KMSDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto top2 = (int64_t)rs1[1] * rs2[1];
			const auto btm2 = (int64_t)rs1[0] * rs2[0];
			bool ov = false;
			const auto res = sat_sub<int64_t, INT32_MIN, INT32_MAX>(
			    &ov, sat_sub<int64_t, INT32_MIN, INT32_MAX>(&ov, rd, top2), btm2);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating Signed Crossed Multiply Two Halfs & Add & Subtract
		case Opcode::KMSXDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto top2 = (int64_t)rs1[1] * rs2[0];
			const auto btm2 = (int64_t)rs1[0] * rs2[1];
			bool ov = false;
			const auto res = sat_sub<int64_t, INT32_MIN, INT32_MAX>(
			    &ov, sat_sub<int64_t, INT32_MIN, INT32_MAX>(&ov, rd, top2), btm2);
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Multiply and Saturating Subtract from 64-Bit Data
		case Opcode::KMSR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			const auto res = rs1 * rs2;
			bool ov = false;
			const auto res2 = sat_add<int64_t>(&ov, res, rd);
			regs.write64x1(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// Saturating Shift Left Logical for Word
		case Opcode::KSLLW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x1F;
			const auto res = rs1 << rs2;
			const auto ov = res > INT32_MAX;
			const auto ov2 = res < INT32_MIN;
			regs.write(instr.rd(), ov ? INT32_MAX : ov2 ? INT32_MIN : (int32_t)res);
			csrs.vxsat.reg |= ov || ov2;
		} break;

		// Saturating Shift Left Logical Immediate for Word
		case Opcode::KSLLIW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = instr.rs2();
			const auto res = rs1 << rs2;
			const auto ov = res > INT32_MAX;
			const auto ov2 = res < INT32_MIN;
			regs.write(instr.rd(), ov ? INT32_MAX : ov2 ? INT32_MIN : (int32_t)res);
			csrs.vxsat.reg |= ov || ov2;
		} break;

		// SIMD 8-bit Saturating Shift Left Logical
		case Opcode::KSLL8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x7;
			std::array<int32_t, 4> res = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 4; ++lane) {
				res[lane] = (uint16_t)rs1[lane] << rs2;
				const auto ov1 = res[lane] > INT8_MAX;
				res[lane] = ov1 ? INT8_MAX : res[lane];
				ov |= ov1;
			}
			regs.write8x4(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 8-bit Saturating Shift Left Logical Immediate
		case Opcode::KSLLI8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = instr.rs2() & 0x7;
			std::array<int32_t, 4> res = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 4; ++lane) {
				res[lane] = (uint16_t)rs1[lane] << rs2;
				const auto ov1 = res[lane] > INT8_MAX;
				res[lane] = ov1 ? INT8_MAX : res[lane];
				ov |= ov1;
			}
			regs.write8x4(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Saturating Shift Left Logical
		case Opcode::KSLL16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0xF;
			std::array<int32_t, 2> res = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 2; ++lane) {
				res[lane] = (uint32_t)rs1[lane] << rs2;
				const auto ov1 = res[lane] > INT16_MAX;
				res[lane] = ov1 ? INT8_MAX : res[lane];
				ov |= ov1;
			}
			regs.write16x2(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Saturating Shift Left Logical Immediate
		case Opcode::KSLLI16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = instr.rs2() & 0xF;
			std::array<int32_t, 2> res = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 2; ++lane) {
				res[lane] = (uint32_t)rs1[lane] << rs2;
				const auto ov1 = res[lane] > INT16_MAX;
				res[lane] = ov1 ? INT8_MAX : res[lane];
				ov |= ov1;
			}
			regs.write16x2(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 8-bit Shift Left Logical with Saturation or Shift Right Arithmetic
		case Opcode::KSLRA8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = (regs.read(instr.rs2()) & 0xF) << (32 - 4) >> (32 - 4);
			std::array<int32_t, 4> res = rs1;
			bool ov = false;
			for (int32_t lane = 0; lane < 4; ++lane) {
				if (rs2) {
					res[lane] = rs2 > 0 ? rs1[lane] << rs2 : rs1[lane] >> -rs2;
					const auto ov1 = res[lane] > INT8_MAX;
					const auto ov2 = res[lane] < INT8_MIN;
					res[lane] = ov1 ? INT8_MAX : ov2 ? INT8_MIN : res[lane];
					ov |= ov1 || ov2;
				}
			}
			regs.write8x4(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 8-bit Shift Left Logical with Saturation or Rounding Shift Right Arithmetic
		case Opcode::KSLRA8_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = (regs.read(instr.rs2()) & 0xF) << (32 - 4) >> (32 - 4);
			std::array<int32_t, 4> res = rs1;
			bool ov = false;
			for (int32_t lane = 0; lane < 4; ++lane) {
				if (rs2) {
					res[lane] = rs2 > 0 ? rs1[lane] << rs2 : ((rs1[lane] >> (-rs2 - 1)) + 1) >> 1;
					const auto ov1 = res[lane] > INT8_MAX;
					const auto ov2 = res[lane] < INT8_MIN;
					res[lane] = ov1 ? INT8_MAX : ov2 ? INT8_MIN : res[lane];
					ov |= ov1 || ov2;
				}
			}
			regs.write8x4(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Shift Left Logical with Saturation or Shift Right Arithmetic
		case Opcode::KSLRA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = (regs.read(instr.rs2()) & 0x1F) << (32 - 5) >> (32 - 5);
			std::array<int32_t, 2> res = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 2; ++lane) {
				res[lane] = rs2 > 0 ? rs1[lane] << rs2 : rs1[lane] >> -rs2;
				const auto ov1 = res[lane] > INT16_MAX;
				const auto ov2 = res[lane] < INT16_MIN;
				res[lane] = ov1 ? INT16_MAX : ov2 ? INT16_MIN : res[lane];
				ov |= ov1 || ov2;
			}
			regs.write16x2(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Shift Left Logical with Saturation or Rounding Shift Right Arithmetic
		case Opcode::KSLRA16_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = (regs.read(instr.rs2()) & 0x1F) << (32 - 5) >> (32 - 5);
			std::array<int32_t, 2> res = {};
			bool ov = false;
			for (int32_t lane = 0; lane < 2; ++lane) {
				res[lane] = rs2 > 0 ? rs1[lane] << rs2 : ((rs1[lane] >> (-rs2 - 1)) + 1) >> 1;
				const auto ov1 = res[lane] > INT16_MAX;
				const auto ov2 = res[lane] < INT16_MIN;
				res[lane] = ov1 ? INT16_MAX : ov2 ? INT16_MIN : res[lane];
				ov |= ov1 || ov2;
			}
			regs.write16x2(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// Shift Left Logical with Q31 Saturation or Shift Right Arithmetic
		case Opcode::KSLRAW: {
			REQUIRE_ISA(P_ISA_EXT);
			const int64_t rs1 = regs.read(instr.rs1());
			const auto rs2 = (regs.read(instr.rs2()) & 0x3F) << (32 - 6) >> (32 - 6);
			int64_t res = 0;
			bool ov = false;
			res = rs2 > 0 ? rs1 << rs2 : rs1 >> -rs2;
			const auto ov1 = res > INT32_MAX;
			const auto ov2 = res < INT32_MIN;
			res = ov1 ? INT32_MAX : ov2 ? INT32_MIN : res;
			ov |= ov1 || ov2;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// Shift Left Logical with Q31 Saturation or Rounding Shift Right Arithmetic
		case Opcode::KSLRAW_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const int64_t rs1 = regs.read(instr.rs1());
			const auto rs2 = (regs.read(instr.rs2()) & 0x3F) << (32 - 6) >> (32 - 6);
			int64_t res = 0;
			bool ov = false;
			res = rs2 > 0 ? rs1 << rs2 : ((rs1 >> (-rs2 - 1)) + 1) >> 1;
			const auto ov1 = res > INT32_MAX;
			const auto ov2 = res < INT32_MIN;
			res = ov1 ? INT32_MAX : ov2 ? INT32_MIN : res;
			ov |= ov1 || ov2;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Signed Saturating Straight Addition & Subtraction
		case Opcode::KSTAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write16x2(instr.rd(), {
			                               sat_sub<int16_t>(&ov, rs1[0], rs2[0]),
			                               sat_add<int16_t>(&ov, rs1[1], rs2[1]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Signed Saturating Straight Subtraction & Addition
		case Opcode::KSTSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write16x2(instr.rd(), {
			                               sat_add<int16_t>(&ov, rs1[0], rs2[0]),
			                               sat_sub<int16_t>(&ov, rs1[1], rs2[1]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 8-bit Signed Saturating Subtraction
		case Opcode::KSUB8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			bool ov = false;
			regs.write8x4(instr.rd(), {
			                              sat_sub<int8_t>(&ov, rs1[0], rs2[0]),
			                              sat_sub<int8_t>(&ov, rs1[1], rs2[1]),
			                              sat_sub<int8_t>(&ov, rs1[2], rs2[2]),
			                              sat_sub<int8_t>(&ov, rs1[3], rs2[3]),
			                          });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Signed Saturating Subtraction
		case Opcode::KSUB16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write16x2(instr.rd(), {
			                               sat_sub<int16_t>(&ov, rs1[0], rs2[0]),
			                               sat_sub<int16_t>(&ov, rs1[1], rs2[1]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// 64-bit Signed Saturating Subtraction
		case Opcode::KSUB64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read64x1(instr.rs1());
			const auto rs2 = regs.read64x1(instr.rs2());
			bool ov = false;
			regs.write64x1(instr.rd(), sat_sub<int64_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Subtraction with Q15 Saturation
		case Opcode::KSUBH: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			bool ov = false;
			regs.write(instr.rd(), sat_sub<int16_t>(&ov, rs1[0], rs2[0]));
			csrs.vxsat.reg |= ov;
		} break;

		// Signed Subtraction with Q31 Saturation
		case Opcode::KSUBW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			bool ov = false;
			regs.write(instr.rd(), sat_sub<int32_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word & Double
		case Opcode::KWMMUL: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto ov = (uint32_t)rs1 == 0x8000'0000 || (uint32_t)rs2 == 0x8000'0000;
			const auto res = ov ? 0x7FFF'FFFF : ((int64_t)rs1 * rs2) >> 31;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD Saturating MSW Signed Multiply Word & Double with Rounding
		case Opcode::KWMMUL_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto ov = (uint32_t)rs1 == 0x8000'0000 || (uint32_t)rs2 == 0x8000'0000;
			const auto res = ov ? 0x7FFF'FFFF : ((((int64_t)rs1 * rs2) >> 30) + 1) >> 1;
			regs.write(instr.rd(), res);
			csrs.vxsat.reg |= ov;
		} break;

		// Multiply and Add to 32-Bit Word
		case Opcode::MADDR32: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read(instr.rd());
			const auto res = (int64_t)rs1 * rs2 + rd;
			regs.write(instr.rd(), res);
		} break;

		// Multiply and Subtract from 32-Bit Word
		case Opcode::MSUBR32: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = (int64_t)regs.read(instr.rd());
			const auto res = rd - (int64_t)rs1 * (int64_t)rs2;
			regs.write(instr.rd(), res);
		} break;

		// Multiply Word Unsigned to 64-bit Data
		case Opcode::MULR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint64_t)regs.read(instr.rs1());
			const auto rs2 = (uint64_t)regs.read(instr.rs2());
			const auto res = rs1 * rs2;
			regs.write64x1(instr.rd(), res);
		} break;

		// Multiply Word Signed to 64-bit Data
		case Opcode::MULSR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = (int64_t)regs.read(instr.rs2());
			const auto res = rs1 * rs2;
			regs.write64x1(instr.rd(), res);
		} break;

		// Parallel Byte Sum of Absolute Difference
		case Opcode::PBSAD: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			uint32_t sum = 0;
			for (int32_t lane = 0; lane < 4; ++lane) {
				sum += std::abs((int32_t)rs1[lane] - (int32_t)rs2[lane]);
			}
			regs.write(instr.rd(), sum);
		} break;

		// Parallel Byte Sum of Absolute Difference Accum
		case Opcode::PBSADA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			auto sum = (uint32_t)regs.read(instr.rd());
			for (int32_t lane = 0; lane < 4; ++lane) {
				sum += std::abs((int32_t)rs1[lane] - (int32_t)rs2[lane]);
			}
			regs.write(instr.rd(), sum);
		} break;

		// Pack Two 16-bit Data from Bottom and Top Half
		case Opcode::PKBT16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs2[1], rs1[0]});
		} break;

		// Pack Two 16-bit Data from Top and Bottom Half
		case Opcode::PKTB16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs2[0], rs1[1]});
		} break;

		// SIMD 8-bit Signed Halving Addition
		case Opcode::RADD8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			auto rd = regs.read8x4(instr.rd());
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = ((int64_t)rs1[lane] + rs2[lane]) >> 1;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Halving Addition
		case Opcode::RADD16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = ((int64_t)rs1[lane] + rs2[lane]) >> 1;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// 64-bit Signed Halving Addition
		case Opcode::RADD64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read64x1(instr.rs1());
			const auto rs2 = regs.read64x1(instr.rs2());
			const auto rd = (rs1 + rs2) >> 1;
			regs.write64x1(instr.rd(), rd);
		} break;

		// 32-bit Signed Halving Addition
		case Opcode::RADDW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = (int64_t)regs.read(instr.rs2());
			const auto rd = (rs1 + rs2) >> 1;
			regs.write(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Halving Cross Addition & Subtraction
		case Opcode::RCRAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs1[0] - rs2[1], rs1[1] + rs2[0]});
		} break;

		// SIMD 16-bit Signed Halving Cross Subtraction & Addition
		case Opcode::RCRSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs1[0] + rs2[1], rs1[1] - rs2[0]});
		} break;

		// SIMD 16-bit Signed Halving Straight Addition & Subtraction
		case Opcode::RSTAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs1[0] - rs2[0], rs1[1] + rs2[1]});
		} break;

		// SIMD 16-bit Signed Halving Straight Subtraction & Addition
		case Opcode::RSTSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs1[0] + rs2[0], rs1[1] - rs2[1]});
		} break;

		// SIMD 8-bit Signed Halving Subtraction
		case Opcode::RSUB8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			auto rd = regs.read8x4(instr.rd());
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = ((int64_t)rs1[lane] - rs2[lane]) >> 1;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Halving Subtraction
		case Opcode::RSUB16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = ((int64_t)rs1[lane] - rs2[lane]) >> 1;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// 64-bit Signed Halving Subtraction
		case Opcode::RSUB64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read64x1(instr.rs1());
			const auto rs2 = regs.read64x1(instr.rs2());
			const auto rd = (rs1 - rs2) >> 1;
			regs.write64x1(instr.rd(), rd);
		} break;

		// 32-bit Signed Halving Subtraction
		case Opcode::RSUBW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (int64_t)regs.read(instr.rs1());
			const auto rs2 = (int64_t)regs.read(instr.rs2());
			const auto rd = (rs1 - rs2) >> 1;
			regs.write(instr.rd(), rd);
		} break;

		// SIMD 8-bit Signed Clip Value
		case Opcode::SCLIP8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto imm = BIT_SLICE(instr.data(), 22, 20);
			const auto max_val = (1 << imm) - 1;
			const auto min_val = -(1 << imm);
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = std::min(std::max(max_val, rs1[lane]), min_val);
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Clip Value
		case Opcode::SCLIP16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto imm = BIT_SLICE(instr.data(), 22, 20);
			const auto max_val = (1 << imm) - 1;
			const auto min_val = -(1 << imm);
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = std::min(std::max(max_val, rs1[lane]), min_val);
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 32-bit Signed Clip Value
		case Opcode::SCLIP32: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto imm = BIT_SLICE(instr.data(), 24, 20);
			const auto max_val = (1 << imm) - 1;
			const auto min_val = -(1 << imm);
			const auto rd = std::min(std::max(max_val, rs1), min_val);
			regs.write(instr.rd(), rd);
		} break;

		// SIMD 8-bit Signed Compare Less Than & Equal
		case Opcode::SCMPLE8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] <= rs2[lane] ? 0xFF : 0;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Compare Less Than & Equal
		case Opcode::SCMPLE16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] <= rs2[lane] ? 0xFFFF : 0;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 8-bit Signed Compare Less Than
		case Opcode::SCMPLT8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] < rs2[lane] ? 0xFF : 0;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Compare Less Than
		case Opcode::SCMPLT16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] < rs2[lane] ? 0xFFFF : 0;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 8-bit Shift Left Logical
		case Opcode::SLL8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x7;
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] << rs2;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-bit Shift Left Logical Immediate
		case Opcode::SLLI8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = instr.rs2() & 0x7;
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] << rs2;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Shift Left Logical
		case Opcode::SLL16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0xF;
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] << rs2;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Shift Left Logical Immediate
		case Opcode::SLLI16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = instr.rs2() & 0xF;
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] << rs2;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// Signed Multiply Halfs & Add 64-bit
		case Opcode::SMAL: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read64x1(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write64x1(instr.rd(), rs1 + rs2[0] * rs2[1]);
		} break;

		// Signed Multiply Bottom Halfs & Add 64-bit
		case Opcode::SMALBB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + rs1[0] * rs2[0]);
		} break;

		// Signed Multiply Bottom Half & Top Half & Add 64-bit
		case Opcode::SMALBT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + rs1[0] * rs2[1]);
		} break;

		// Signed Multiply Two Halfs and Two Adds 64-bit
		case Opcode::SMALDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + rs1[1] * rs2[1] + rs1[0] * rs2[0]);
		} break;

		// Signed Crossed Multiply Two Halfs and Two Adds 64-bit
		case Opcode::SMALXDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + rs1[0] * rs2[1] + rs1[1] * rs2[0]);
		} break;

		// Signed Multiply Two Halfs & Subtract & Add 64-bit
		case Opcode::SMALDS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + (rs1[1] * rs2[1] - rs1[0] * rs2[0]));
		} break;

		// Signed Multiply Two Halfs & Reverse Subtract & Add 64-bit
		case Opcode::SMALDRS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + (rs1[0] * rs2[0] - rs1[1] * rs2[1]));
		} break;

		// Signed Crossed Multiply Two Halfs & Subtract & Add 64-bit
		case Opcode::SMALXDS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + (rs1[1] * rs2[0] - rs1[0] * rs2[1]));
		} break;

		// Signed Multiply and Add to 64-Bit Data
		case Opcode::SMAR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd + rs1 * rs2);
		} break;

		// Signed Multiply Four Bytes with 32-bit Adds
		case Opcode::SMAQA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			const auto rd = regs.read(instr.rd());
			int64_t sum = 0;
			for (int32_t lane = 0; lane < 4; ++lane) {
				sum += (int64_t)rs1[lane] * (int64_t)rs2[lane];
			}
			regs.write(instr.rd(), rd + sum);
		} break;

		// Signed and Unsigned Multiply Four Bytes with 32-bit Adds
		case Opcode::SMAQA_SU: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			const auto rd = regs.read(instr.rd());
			int64_t sum = 0;
			for (int32_t lane = 0; lane < 4; ++lane) {
				sum += (int64_t)rs1[lane] * (uint64_t)rs2[lane];
			}
			regs.write(instr.rd(), rd + sum);
		} break;

		// SIMD 8-bit Signed Maximum
		case Opcode::SMAX8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = std::max(rs1[lane], rs2[lane]);
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Maximum
		case Opcode::SMAX16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = std::max(rs1[lane], rs2[lane]);
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD Signed Multiply Bottom Half & Bottom Half
		case Opcode::SMBB16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write(instr.rd(), rs1[0] * rs2[0]);
		} break;

		// SIMD Signed Multiply Bottom Half & Top Half
		case Opcode::SMBT16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write(instr.rd(), rs1[0] * rs2[1]);
		} break;

		// SIMD Signed Multiply Top Half & Top Half
		case Opcode::SMTT16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write(instr.rd(), rs1[1] * rs2[1]);
		} break;

		// SIMD Signed Multiply Two Halfs and Subtract
		case Opcode::SMDS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write(instr.rd(), rs1[1] * rs2[1] - rs1[0] * rs2[0]);
		} break;

		// SIMD Signed Multiply Two Halfs and Reverse Subtract
		case Opcode::SMDRS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write(instr.rd(), rs1[0] * rs2[0] - rs1[1] * rs2[1]);
		} break;

		// SIMD Signed Crossed Multiply Two Halfs and Subtract
		case Opcode::SMXDS: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write(instr.rd(), rs1[1] * rs2[0] - rs1[0] * rs2[1]);
		} break;

		// SIMD 8-bit Signed Minimum
		case Opcode::SMIN8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = std::min(rs1[lane], rs2[lane]);
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Signed Minimum
		case Opcode::SMIN16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = std::min(rs1[lane], rs2[lane]);
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD MSW Signed Multiply Word
		case Opcode::SMMUL: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = (int64_t)rs1 * rs2;
			regs.write(instr.rd(), rd >> 32);
		} break;

		// SIMD MSW Signed Multiply Word with Rounding
		case Opcode::SMMUL_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = (int64_t)rs1 * rs2;
			regs.write(instr.rd(), ((rd >> 31) + 1) >> 1);
		} break;

		// SIMD MSW Signed Multiply Word and Bottom Half
		case Opcode::SMMWB: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = (int64_t)rs1 * rs2[0];
			regs.write(instr.rd(), rd >> 16);
		} break;

		// SIMD MSW Signed Multiply Word and Bottom Half with Rounding
		case Opcode::SMMWB_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = (int64_t)rs1 * rs2[0];
			regs.write(instr.rd(), ((rd >> 15) + 1) >> 1);
		} break;

		// SIMD MSW Signed Multiply Word and Top Half
		case Opcode::SMMWT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = (int64_t)rs1 * rs2[1];
			regs.write(instr.rd(), rd >> 16);
		} break;

		// SIMD MSW Signed Multiply Word and Top Half with Rounding
		case Opcode::SMMWT_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = (int64_t)rs1 * rs2[1];
			regs.write(instr.rd(), ((rd >> 15) + 1) >> 1);
		} break;

		// Signed Multiply Two Halfs & Add & Subtract 64-bit
		case Opcode::SMSLDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd - rs1[1] * rs2[1] - rs1[0] * rs2[0]);
		} break;

		// Signed Crossed Multiply Two Halfs & Add & Subtract 64-bit
		case Opcode::SMSLXDA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd - rs1[0] * rs2[1] - rs1[1] * rs2[0]);
		} break;

		// Signed Multiply and Subtract from 64-Bit Data
		case Opcode::SMSR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2());
			const auto rd = regs.read64x1(instr.rd());
			regs.write64x1(instr.rd(), rd - rs1 * rs2);
		} break;

		// SIMD Signed 8-bit Multiply
		case Opcode::SMUL8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			regs.write8x4(instr.rd(), {
			                              rs1[0] * rs2[0],
			                              rs1[1] * rs2[1],
			                              rs1[2] * rs2[2],
			                              rs1[3] * rs2[3],
			                          });
		} break;

		// SIMD Signed Crossed 8-bit Multiply
		case Opcode::SMULX8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			regs.write8x4(instr.rd(), {
			                              rs1[0] * rs2[1],
			                              rs1[1] * rs2[0],
			                              rs1[2] * rs2[3],
			                              rs1[3] * rs2[2],
			                          });
		} break;

		// SIMD Signed 16-bit Multiply
		case Opcode::SMUL16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {
			                               rs1[0] * rs2[0],
			                               rs1[1] * rs2[1],
			                           });
		} break;

		// SIMD Signed Crossed 16-bit Multiply
		case Opcode::SMULX16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {
			                               rs1[0] * rs2[1],
			                               rs1[1] * rs2[0],
			                           });
		} break;

		// Rounding Shift Right Arithmetic
		case Opcode::SRA_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x1F;
			if (rs2)
				regs.write(instr.rd(), ((rs1 >> (rs1 - 1)) + 1) >> 1);
			else
				regs.write(instr.rd(), rs1);
		} break;

		// Rounding Shift Right Arithmetic Immediate
		case Opcode::SRAI_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read(instr.rs1());
			const auto rs2 = instr.rs2() & 0x1F;
			if (rs2)
				regs.write(instr.rd(), ((rs1 >> (rs1 - 1)) + 1) >> 1);
			else
				regs.write(instr.rd(), rs1);
		} break;

		// SIMD 8-bit Shift Right Arithmetic
		case Opcode::SRA8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x07;
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-bit Rounding Shift Right Arithmetic
		case Opcode::SRA8_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x07;
			std::array<int32_t, 4> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 4; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-bit Shift Right Arithmetic Immediate
		case Opcode::SRAI8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = instr.rs2() & 0x07;
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-bit Rounding Shift Right Arithmetic Immediate
		case Opcode::SRAI8_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = instr.rs2() & 0x07;
			std::array<int32_t, 4> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 4; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Shift Right Arithmetic
		case Opcode::SRA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x0F;
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Rounding Shift Right Arithmetic
		case Opcode::SRA16_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x0F;
			std::array<int32_t, 2> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 2; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Shift Right Arithmetic Immediate
		case Opcode::SRAI16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = instr.rs2() & 0x0F;
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Rounding Shift Right Arithmetic Immediate
		case Opcode::SRAI16_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = instr.rs2() & 0x0F;
			std::array<int32_t, 2> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 2; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 8-bit Shift Right Logical
		case Opcode::SRL8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x07;
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-bit Rounding Shift Right Logical
		case Opcode::SRL8_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x07;
			std::array<int32_t, 4> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 4; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-bit Shift Right Logical Immediate
		case Opcode::SRLI8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = instr.rs2() & 0x07;
			std::array<int32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 8-bit Rounding Shift Right Logical Immediate
		case Opcode::SRLI8_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = instr.rs2() & 0x07;
			std::array<int32_t, 4> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 4; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Shift Right Logical
		case Opcode::SRL16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x0F;
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Rounding Shift Right Logical
		case Opcode::SRL16_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.read(instr.rs2()) & 0x0F;
			std::array<int32_t, 2> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 2; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Shift Right Logical Immediate
		case Opcode::SRLI16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = instr.rs2() & 0x0F;
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] >> rs2;
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Rounding Shift Right Logical Immediate
		case Opcode::SRLI16_u: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = instr.rs2() & 0x0F;
			std::array<int32_t, 2> rd = {};
			if (rs2)
				for (int32_t lane = 0; lane < 2; ++lane) {
					rd[lane] = ((rs1[lane] >> (rs2 - 1)) + 1) >> 1;
				}
			regs.write16x2(instr.rd(), rd);
		} break;

		// SIMD 16-bit Straight Addition & Subtraction
		case Opcode::STAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs1[0] + rs2[0], rs1[1] - rs2[1]});
		} break;

		// SIMD 16-bit Straight Subtraction & Addition
		case Opcode::STSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			regs.write16x2(instr.rd(), {rs1[0] - rs2[0], rs1[1] + rs2[1]});
		} break;

		// SIMD 8-bit Subtraction
		case Opcode::SUB8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			const auto rs2 = regs.read8x4(instr.rs2());
			auto rd = regs.read8x4(instr.rd());
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] - rs2[lane];
			}
			regs.write8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Subtraction
		case Opcode::SUB16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			const auto rs2 = regs.read16x2(instr.rs2());
			std::array<int32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] - rs2[lane];
			}
			regs.write16x2(instr.rd(), rd);
		} break;

		// 64-bit Subtraction
		case Opcode::SUB64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read64x1(instr.rs1());
			const auto rs2 = regs.read64x1(instr.rs2());
			const auto rd = rs1 - rs2;
			regs.write64x1(instr.rd(), rd);
		} break;

		// Signed Unpacking Bytes 1 & 0
		case Opcode::SUNPKD810: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			regs.write16x2(instr.rd(), {rs1[0], rs1[1]});
		} break;

		// Signed Unpacking Bytes 2 & 0
		case Opcode::SUNPKD820: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			regs.write16x2(instr.rd(), {rs1[0], rs1[2]});
		} break;

		// Signed Unpacking Bytes 3 & 0
		case Opcode::SUNPKD830: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			regs.write16x2(instr.rd(), {rs1[0], rs1[3]});
		} break;

		// Signed Unpacking Bytes 3 & 1
		case Opcode::SUNPKD831: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			regs.write16x2(instr.rd(), {rs1[1], rs1[3]});
		} break;

		// Signed Unpacking Bytes 3 & 2
		case Opcode::SUNPKD832: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			regs.write16x2(instr.rd(), {rs1[2], rs1[3]});
		} break;

		// Swap Byte within Halfword
		case Opcode::SWAP8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read8x4(instr.rs1());
			regs.write8x4(instr.rd(), {rs1[3], rs1[2], rs1[1], rs1[0]});
		} break;

		// Swap Halfword within Word
		case Opcode::SWAP16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.read16x2(instr.rs1());
			regs.write16x2(instr.rd(), {rs1[1], rs1[0]});
		} break;

		// SIMD 8-bit Unsigned Clip Value
		case Opcode::UCLIP8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto imm = (uint32_t)BIT_SLICE(instr.data(), 22, 20);
			const auto max_val = (uint32_t)(1 << imm) - 1u;
			const auto min_val = (uint32_t)0;
			std::array<uint32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = std::min(std::max(max_val, rs1[lane]), min_val);
			}
			regs.uwrite8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Clip Value
		case Opcode::UCLIP16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto imm = (uint32_t)BIT_SLICE(instr.data(), 22, 20);
			const auto max_val = (uint32_t)(1 << imm) - 1u;
			const auto min_val = (uint32_t)0;
			std::array<uint32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = std::min(std::max(max_val, rs1[lane]), min_val);
			}
			regs.uwrite16x2(instr.rd(), rd);
		} break;

		// SIMD 32-bit Unsigned Clip Value
		case Opcode::UCLIP32: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread(instr.rs1());
			const auto imm = (uint32_t)BIT_SLICE(instr.data(), 24, 20);
			const auto max_val = (uint32_t)(1 << imm) - 1u;
			const auto min_val = (uint32_t)0;
			const auto rd = std::min(std::max(max_val, rs1), min_val);
			regs.uwrite(instr.rd(), rd);
		} break;

		// SIMD 8-bit Unsigned Compare Less Than & Equal
		case Opcode::UCMPLE8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			std::array<uint32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] <= rs2[lane] ? 0xFF : 0;
			}
			regs.uwrite8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Compare Less Than & Equal
		case Opcode::UCMPLE16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			std::array<uint32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] <= rs2[lane] ? 0xFFFF : 0;
			}
			regs.uwrite16x2(instr.rd(), rd);
		} break;

		// SIMD 8-bit Unsigned Compare Less Than
		case Opcode::UCMPLT8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			std::array<uint32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = rs1[lane] < rs2[lane] ? 0xFF : 0;
			}
			regs.uwrite8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Compare Less Than
		case Opcode::UCMPLT16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			std::array<uint32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = rs1[lane] < rs2[lane] ? 0xFFFF : 0;
			}
			regs.uwrite16x2(instr.rd(), rd);
		} break;

		// SIMD 8-bit Unsigned Saturating Addition
		case Opcode::UKADD8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			bool ov = false;
			regs.uwrite8x4(instr.rd(), {
			                               sat_uadd<uint8_t>(&ov, rs1[0], rs2[0]),
			                               sat_uadd<uint8_t>(&ov, rs1[1], rs2[1]),
			                               sat_uadd<uint8_t>(&ov, rs1[2], rs2[2]),
			                               sat_uadd<uint8_t>(&ov, rs1[3], rs2[3]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Unsigned Saturating Addition
		case Opcode::UKADD16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite16x2(instr.rd(), {
			                                sat_uadd<uint16_t>(&ov, rs1[0], rs2[0]),
			                                sat_uadd<uint16_t>(&ov, rs1[1], rs2[1]),
			                            });
			csrs.vxsat.reg |= ov;
		} break;

		// 64-bit Unsigned Saturating Addition
		case Opcode::UKADD64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread64x1(instr.rs1());
			const auto rs2 = regs.uread64x1(instr.rs2());
			bool ov = false;
			regs.uwrite64x1(instr.rd(), sat_uadd<uint64_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// Unsigned Addition with Q15 Saturation
		case Opcode::UKADDH: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite(instr.rd(), sat_uadd<uint16_t>(&ov, rs1[0], rs2[0]));
			csrs.vxsat.reg |= ov;
		} break;

		// Unsigned Addition with Q31 Saturation
		case Opcode::UKADDW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread(instr.rs1());
			const auto rs2 = regs.uread(instr.rs2());
			bool ov = false;
			regs.uwrite(instr.rd(), sat_uadd<uint32_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Unsigned Saturating Cross Addition & Subtractions
		case Opcode::UKCRAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite16x2(instr.rd(), {
			                                sat_usub<uint16_t>(&ov, rs1[0], rs2[1]),
			                                sat_uadd<uint16_t>(&ov, rs1[1], rs2[0]),
			                            });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Unsigned Saturating Cross Subtraction & Addition
		case Opcode::UKCRSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite16x2(instr.rd(), {
			                                sat_uadd<uint16_t>(&ov, rs1[0], rs2[1]),
			                                sat_usub<uint16_t>(&ov, rs1[1], rs2[0]),
			                            });
			csrs.vxsat.reg |= ov;
		} break;

		// Unsigned Multiply and Saturating Add to 64-Bit Data
		case Opcode::UKMAR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint32_t)regs.uread(instr.rs1());
			const auto rs2 = (uint32_t)regs.uread(instr.rs2());
			const auto rd = (uint64_t)regs.uread64x1(instr.rd());
			const auto res = rs1 * rs2;
			bool ov = false;
			const auto res2 = sat_uadd<uint64_t>(&ov, res, rd);
			regs.uwrite64x1(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// Unsigned Multiply and Saturating Subtract from 64-Bit Data
		case Opcode::UKMSR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint32_t)regs.uread(instr.rs1());
			const auto rs2 = (uint32_t)regs.uread(instr.rs2());
			const auto rd = (uint64_t)regs.uread64x1(instr.rd());
			const auto res = rs1 * rs2;
			bool ov = false;
			const auto res2 = sat_uadd<uint64_t>(&ov, res, rd);
			regs.uwrite64x1(instr.rd(), res2);
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Unsigned Saturating Straight Addition & Subtraction
		case Opcode::UKSTAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite16x2(instr.rd(), {
			                                sat_usub<uint16_t>(&ov, rs1[0], rs2[0]),
			                                sat_uadd<uint16_t>(&ov, rs1[1], rs2[1]),
			                            });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Unsigned Saturating Straight Subtraction & Addition
		case Opcode::UKSTSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite16x2(instr.rd(), {
			                                sat_uadd<uint16_t>(&ov, rs1[0], rs2[0]),
			                                sat_usub<uint16_t>(&ov, rs1[1], rs2[1]),
			                            });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 8-bit Unsigned Saturating Subtraction
		case Opcode::UKSUB8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			bool ov = false;
			regs.uwrite8x4(instr.rd(), {
			                               sat_usub<uint8_t>(&ov, rs1[0], rs2[0]),
			                               sat_usub<uint8_t>(&ov, rs1[1], rs2[1]),
			                               sat_usub<uint8_t>(&ov, rs1[2], rs2[2]),
			                               sat_usub<uint8_t>(&ov, rs1[3], rs2[3]),
			                           });
			csrs.vxsat.reg |= ov;
		} break;

		// SIMD 16-bit Unsigned Saturating Subtraction
		case Opcode::UKSUB16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite16x2(instr.rd(), {
			                                sat_usub<uint16_t>(&ov, rs1[0], rs2[0]),
			                                sat_usub<uint16_t>(&ov, rs1[1], rs2[1]),
			                            });
			csrs.vxsat.reg |= ov;
		} break;

		// 64-bit Unsigned Saturating Subtraction
		case Opcode::UKSUB64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint64_t)regs.uread64x1(instr.rs1());
			const auto rs2 = (uint64_t)regs.uread64x1(instr.rs2());
			bool ov = false;
			regs.uwrite64x1(instr.rd(), sat_usub<uint64_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// Unsigned Subtraction with U16 Saturation
		case Opcode::UKSUBH: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			bool ov = false;
			regs.uwrite(instr.rd(), sat_usub<uint16_t>(&ov, rs1[0], rs2[0]));
			csrs.vxsat.reg |= ov;
		} break;

		// Unsigned Subtraction with U32 Saturation
		case Opcode::UKSUBW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint32_t)regs.uread(instr.rs1());
			const auto rs2 = (uint32_t)regs.uread(instr.rs2());
			bool ov = false;
			regs.uwrite(instr.rd(), sat_usub<uint32_t>(&ov, rs1, rs2));
			csrs.vxsat.reg |= ov;
		} break;

		// Unsigned Multiply and Add to 64-Bit Data
		case Opcode::UMAR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread(instr.rs1());
			const auto rs2 = regs.uread(instr.rs2());
			const auto rd = regs.uread64x1(instr.rd());
			const auto res = rs1 * rs2;
			const auto res2 = res + rd;
			regs.write64x1(instr.rd(), res2);
		} break;

		// Unsigned Multiply Four Bytes with 32-bit Adds
		case Opcode::UMAQA: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			const auto rd = regs.uread(instr.rd());
			uint64_t sum = 0;
			for (int32_t lane = 0; lane < 4; ++lane) {
				sum += (uint64_t)rs1[lane] * (uint64_t)rs2[lane];
			}
			regs.uwrite(instr.rd(), rd + sum);
		} break;

		// SIMD 8-bit Unsigned Maximum
		case Opcode::UMAX8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			std::array<uint32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = std::max(rs1[lane], rs2[lane]);
			}
			regs.uwrite8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Maximum
		case Opcode::UMAX16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			std::array<uint32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = std::max(rs1[lane], rs2[lane]);
			}
			regs.uwrite16x2(instr.rd(), rd);
		} break;

		// SIMD 8-bit Unsigned Minimum
		case Opcode::UMIN8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			std::array<uint32_t, 4> rd = {};
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = std::min(rs1[lane], rs2[lane]);
			}
			regs.uwrite8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Minimum
		case Opcode::UMIN16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			std::array<uint32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = std::min(rs1[lane], rs2[lane]);
			}
			regs.uwrite16x2(instr.rd(), rd);
		} break;

		// Unsigned Multiply and Subtract from 64-Bit Data
		case Opcode::UMSR64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread(instr.rs1());
			const auto rs2 = regs.uread(instr.rs2());
			const auto rd = regs.uread64x1(instr.rd());
			regs.uwrite64x1(instr.rd(), rd - rs1 * rs2);
		} break;

		// SIMD Unsigned 8-bit Multiply
		case Opcode::UMUL8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			regs.uwrite8x4(instr.rd(), {
			                               rs1[0] * rs2[0],
			                               rs1[1] * rs2[1],
			                               rs1[2] * rs2[2],
			                               rs1[3] * rs2[3],
			                           });
		} break;

		// SIMD Unsigned Crossed 8-bit Multiply
		case Opcode::UMULX8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			regs.uwrite8x4(instr.rd(), {
			                               rs1[0] * rs2[1],
			                               rs1[1] * rs2[0],
			                               rs1[2] * rs2[3],
			                               rs1[3] * rs2[2],
			                           });
		} break;

		// SIMD Unsigned 16-bit Multiply
		case Opcode::UMUL16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			regs.uwrite16x2(instr.rd(), {
			                                rs1[0] * rs2[0],
			                                rs1[1] * rs2[1],
			                            });
		} break;

		// SIMD Unsigned Crossed 16-bit Multiply
		case Opcode::UMULX16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			regs.uwrite16x2(instr.rd(), {
			                                rs1[0] * rs2[1],
			                                rs1[1] * rs2[0],
			                            });
		} break;

		// SIMD 8-bit Unsigned Halving Addition
		case Opcode::URADD8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			auto rd = regs.uread8x4(instr.rd());
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = ((uint64_t)rs1[lane] + rs2[lane]) >> 1;
			}
			regs.uwrite8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Halving Addition
		case Opcode::URADD16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			std::array<uint32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = ((uint64_t)rs1[lane] + rs2[lane]) >> 1;
			}

			regs.uwrite16x2(instr.rd(), rd);
		} break;

		// 64-bit Unsigned Halving Addition
		case Opcode::URADD64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint64_t)regs.uread64x1(instr.rs1());
			const auto rs2 = (uint64_t)regs.uread64x1(instr.rs2());
			const auto rd = (rs1 + rs2) >> 1;
			regs.uwrite64x1(instr.rd(), rd);
		} break;

		// 32-bit Unsigned Halving Addition
		case Opcode::URADDW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint64_t)regs.uread(instr.rs1());
			const auto rs2 = (uint64_t)regs.uread(instr.rs2());
			const auto rd = (rs1 + rs2) >> 1;
			regs.uwrite(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Halving Cross Addition & Subtraction
		case Opcode::URCRAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			regs.uwrite16x2(instr.rd(), {rs1[0] - rs2[1], rs1[1] + rs2[0]});
		} break;

		// SIMD 16-bit Unsigned Halving Cross Subtraction & Addition
		case Opcode::URCRSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			regs.uwrite16x2(instr.rd(), {rs1[0] + rs2[1], rs1[1] - rs2[0]});
		} break;

		// SIMD 16-bit Unsigned Halving Straight Addition & Subtraction
		case Opcode::URSTAS16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			regs.uwrite16x2(instr.rd(), {rs1[0] - rs2[0], rs1[1] + rs2[1]});
		} break;

		// SIMD 16-bit Unsigned Halving Straight Subtraction & Addition
		case Opcode::URSTSA16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			regs.uwrite16x2(instr.rd(), {rs1[0] + rs2[0], rs1[1] - rs2[1]});
		} break;

		// SIMD 8-bit Unsigned Halving Subtraction
		case Opcode::URSUB8: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			const auto rs2 = regs.uread8x4(instr.rs2());
			auto rd = regs.uread8x4(instr.rd());
			for (int32_t lane = 0; lane < 4; ++lane) {
				rd[lane] = ((uint64_t)rs1[lane] - rs2[lane]) >> 1;
			}
			regs.uwrite8x4(instr.rd(), rd);
		} break;

		// SIMD 16-bit Unsigned Halving Subtraction
		case Opcode::URSUB16: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread16x2(instr.rs1());
			const auto rs2 = regs.uread16x2(instr.rs2());
			std::array<uint32_t, 2> rd = {};
			for (int32_t lane = 0; lane < 2; ++lane) {
				rd[lane] = ((uint64_t)rs1[lane] - rs2[lane]) >> 1;
			}
			regs.uwrite16x2(instr.rd(), rd);
		} break;

		// 64-bit Unsigned Halving Subtraction
		case Opcode::URSUB64: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread64x1(instr.rs1());
			const auto rs2 = regs.uread64x1(instr.rs2());
			const auto rd = (rs1 - rs2) >> 1;
			regs.uwrite64x1(instr.rd(), rd);
		} break;

		// 32-bit Unsigned Halving Subtraction
		case Opcode::URSUBW: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = (uint64_t)regs.uread(instr.rs1());
			const auto rs2 = (uint64_t)regs.uread(instr.rs2());
			const auto rd = (rs1 - rs2) >> 1;
			regs.uwrite(instr.rd(), rd);
		} break;

		// Extract Word from 64-bit Immediate
		case Opcode::WEXTI: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread64x1(instr.rs1());
			regs.uwrite(instr.rd(), rs1 >> instr.rs2());
		} break;

		// Extract Word from 64-bit
		case Opcode::WEXT: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread64x1(instr.rs1());
			const auto rs2 = regs.uread(instr.rs2()) & 0x1F;
			regs.uwrite(instr.rd(), rs1 >> rs2);
		} break;

		// Unsigned Unpacking Bytes 1 & 0
		case Opcode::ZUNPKD810: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			regs.uwrite16x2(instr.rd(), {rs1[0], rs1[1]});
		} break;

		// Unsigned Unpacking Bytes 2 & 0
		case Opcode::ZUNPKD820: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			regs.uwrite16x2(instr.rd(), {rs1[0], rs1[2]});
		} break;

		// Unsigned Unpacking Bytes 3 & 0
		case Opcode::ZUNPKD830: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			regs.uwrite16x2(instr.rd(), {rs1[0], rs1[3]});
		} break;

		// Unigned Unpacking Bytes 3 & 1
		case Opcode::ZUNPKD831: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			regs.uwrite16x2(instr.rd(), {rs1[1], rs1[3]});
		} break;

		// Unsigned Unpacking Bytes 3 & 2
		case Opcode::ZUNPKD832: {
			REQUIRE_ISA(P_ISA_EXT);
			const auto rs1 = regs.uread8x4(instr.rs1());
			regs.uwrite16x2(instr.rd(), {rs1[2], rs1[3]});
		} break;

		// instructions accepted by decoder but not by this RV32IMACFP ISS -> do normal trap
		// RV64I
		case Opcode::LWU:
		case Opcode::LD:
		case Opcode::SD:
		case Opcode::ADDIW:
		case Opcode::SLLIW:
		case Opcode::SRLIW:
		case Opcode::SRAIW:
		case Opcode::ADDW:
		case Opcode::SUBW:
		case Opcode::SLLW:
		case Opcode::SRLW:
		case Opcode::SRAW:
			// RV64M
		case Opcode::MULW:
		case Opcode::DIVW:
		case Opcode::DIVUW:
		case Opcode::REMW:
		case Opcode::REMUW:
			// RV64A
		case Opcode::LR_D:
		case Opcode::SC_D:
		case Opcode::AMOSWAP_D:
		case Opcode::AMOADD_D:
		case Opcode::AMOXOR_D:
		case Opcode::AMOAND_D:
		case Opcode::AMOOR_D:
		case Opcode::AMOMIN_D:
		case Opcode::AMOMAX_D:
		case Opcode::AMOMINU_D:
		case Opcode::AMOMAXU_D:
			// RV64F
		case Opcode::FCVT_L_S:
		case Opcode::FCVT_LU_S:
		case Opcode::FCVT_S_L:
		case Opcode::FCVT_S_LU:
			// RV64D
		case Opcode::FCVT_L_D:
		case Opcode::FCVT_LU_D:
		case Opcode::FMV_X_D:
		case Opcode::FCVT_D_L:
		case Opcode::FCVT_D_LU:
		case Opcode::FMV_D_X:
			RAISE_ILLEGAL_INSTRUCTION();
			break;

		default:
			throw std::runtime_error("unknown opcode");
	}
}

uint64_t ISS::_compute_and_get_current_cycles() {
	assert(cycle_counter % cycle_time == sc_core::SC_ZERO_TIME);
	assert(cycle_counter.value() % cycle_time.value() == 0);

	uint64_t num_cycles = cycle_counter.value() / cycle_time.value();

	return num_cycles;
}

bool ISS::is_invalid_csr_access(uint32_t csr_addr, bool is_write) {
	if (csr_addr == csr::FFLAGS_ADDR || csr_addr == csr::FRM_ADDR || csr_addr == csr::FCSR_ADDR) {
		REQUIRE_ISA(F_ISA_EXT);
	}
	PrivilegeLevel csr_prv = (0x300 & csr_addr) >> 8;
	bool csr_readonly = ((0xC00 & csr_addr) >> 10) == 3;
	bool s_invalid = (csr_prv == SupervisorMode) && !csrs.misa.has_supervisor_mode_extension();
	bool u_invalid = (csr_prv == UserMode) && !csrs.misa.has_user_mode_extension();
	return (is_write && csr_readonly) || (prv < csr_prv) || s_invalid || u_invalid;
}

void ISS::validate_csr_counter_read_access_rights(uint32_t addr) {
	// match against counter CSR addresses, see RISC-V privileged spec for the address definitions
	if ((addr >= 0xC00 && addr <= 0xC1F) || (addr >= 0xC80 && addr <= 0xC9F)) {
		auto cnt = addr & 0x1F;  // 32 counter in total, naturally aligned with the mcounteren and scounteren CSRs

		if (s_mode() && !csr::is_bitset(csrs.mcounteren, cnt))
			RAISE_ILLEGAL_INSTRUCTION();

		if (u_mode() && (!csr::is_bitset(csrs.mcounteren, cnt) || !csr::is_bitset(csrs.scounteren, cnt)))
			RAISE_ILLEGAL_INSTRUCTION();
	}
}

uint32_t ISS::get_csr_value(uint32_t addr) {
	validate_csr_counter_read_access_rights(addr);

	auto read = [=](auto &x, uint32_t mask) { return x.reg & mask; };

	using namespace csr;

	switch (addr) {
		case TIME_ADDR:
		case MTIME_ADDR: {
			uint64_t mtime = clint->update_and_get_mtime();
			csrs.time.reg = mtime;
			return csrs.time.words.low;
		}

		case TIMEH_ADDR:
		case MTIMEH_ADDR: {
			uint64_t mtime = clint->update_and_get_mtime();
			csrs.time.reg = mtime;
			return csrs.time.words.high;
		}

		case MCYCLE_ADDR:
			csrs.cycle.reg = _compute_and_get_current_cycles();
			return csrs.cycle.words.low;

		case MCYCLEH_ADDR:
			csrs.cycle.reg = _compute_and_get_current_cycles();
			return csrs.cycle.words.high;

		case MINSTRET_ADDR:
			return csrs.instret.words.low;

		case MINSTRETH_ADDR:
			return csrs.instret.words.high;

		SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32:  // not implemented
			return 0;

		case MSTATUS_ADDR:
			return read(csrs.mstatus, MSTATUS_MASK);
		case SSTATUS_ADDR:
			return read(csrs.mstatus, SSTATUS_MASK);
		case USTATUS_ADDR:
			return read(csrs.mstatus, USTATUS_MASK);

		case MIP_ADDR:
			return read(csrs.mip, MIP_READ_MASK);
		case SIP_ADDR:
			return read(csrs.mip, SIP_MASK);
		case UIP_ADDR:
			return read(csrs.mip, UIP_MASK);

		case MIE_ADDR:
			return read(csrs.mie, MIE_MASK);
		case SIE_ADDR:
			return read(csrs.mie, SIE_MASK);
		case UIE_ADDR:
			return read(csrs.mie, UIE_MASK);

		case SATP_ADDR:
			if (csrs.mstatus.fields.tvm)
				RAISE_ILLEGAL_INSTRUCTION();
			break;

		case FCSR_ADDR:
			return read(csrs.fcsr, FCSR_MASK);

		case FFLAGS_ADDR:
			return csrs.fcsr.fields.fflags;

		case FRM_ADDR:
			return csrs.fcsr.fields.frm;

		// debug CSRs not supported, thus hardwired
		case TSELECT_ADDR:
			return 1;  // if a zero write by SW is preserved, then debug mode is supported (thus hardwire to non-zero)
		case TDATA1_ADDR:
		case TDATA2_ADDR:
		case TDATA3_ADDR:
		case DCSR_ADDR:
		case DPC_ADDR:
		case DSCRATCH0_ADDR:
		case DSCRATCH1_ADDR:
			return 0;
	}

	if (!csrs.is_valid_csr32_addr(addr))
		RAISE_ILLEGAL_INSTRUCTION();

	return csrs.default_read32(addr);
}

void ISS::set_csr_value(uint32_t addr, uint32_t value) {
	auto write = [=](auto &x, uint32_t mask) { x.reg = (x.reg & ~mask) | (value & mask); };

	using namespace csr;

	switch (addr) {
		case MISA_ADDR:                         // currently, read-only, thus cannot be changed at runtime
		SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32:  // not implemented
			break;

		case SATP_ADDR: {
			if (csrs.mstatus.fields.tvm)
				RAISE_ILLEGAL_INSTRUCTION();
			write(csrs.satp, SATP_MASK);
			// std::cout << "[iss] satp=" << boost::format("%x") % csrs.satp.reg << std::endl;
		} break;

		case MTVEC_ADDR:
			write(csrs.mtvec, MTVEC_MASK);
			break;
		case STVEC_ADDR:
			write(csrs.stvec, MTVEC_MASK);
			break;
		case UTVEC_ADDR:
			write(csrs.utvec, MTVEC_MASK);
			break;

		case MEPC_ADDR:
			write(csrs.mepc, pc_alignment_mask());
			break;
		case SEPC_ADDR:
			write(csrs.sepc, pc_alignment_mask());
			break;
		case UEPC_ADDR:
			write(csrs.uepc, pc_alignment_mask());
			break;

		case MSTATUS_ADDR:
			write(csrs.mstatus, MSTATUS_MASK);
			break;
		case SSTATUS_ADDR:
			write(csrs.mstatus, SSTATUS_MASK);
			break;
		case USTATUS_ADDR:
			write(csrs.mstatus, USTATUS_MASK);
			break;

		case MIP_ADDR:
			write(csrs.mip, MIP_WRITE_MASK);
			break;
		case SIP_ADDR:
			write(csrs.mip, SIP_MASK);
			break;
		case UIP_ADDR:
			write(csrs.mip, UIP_MASK);
			break;

		case MIE_ADDR:
			write(csrs.mie, MIE_MASK);
			break;
		case SIE_ADDR:
			write(csrs.mie, SIE_MASK);
			break;
		case UIE_ADDR:
			write(csrs.mie, UIE_MASK);
			break;

		case MIDELEG_ADDR:
			write(csrs.mideleg, MIDELEG_MASK);
			break;

		case MEDELEG_ADDR:
			write(csrs.medeleg, MEDELEG_MASK);
			break;

		case SIDELEG_ADDR:
			write(csrs.sideleg, SIDELEG_MASK);
			break;

		case SEDELEG_ADDR:
			write(csrs.sedeleg, SEDELEG_MASK);
			break;

		case MCOUNTEREN_ADDR:
			write(csrs.mcounteren, MCOUNTEREN_MASK);
			break;

		case SCOUNTEREN_ADDR:
			write(csrs.scounteren, MCOUNTEREN_MASK);
			break;

		case MCOUNTINHIBIT_ADDR:
			write(csrs.mcountinhibit, MCOUNTINHIBIT_MASK);
			break;

		case FCSR_ADDR:
			write(csrs.fcsr, FCSR_MASK);
			break;

		case FFLAGS_ADDR:
			csrs.fcsr.fields.fflags = value;
			break;

		case FRM_ADDR:
			csrs.fcsr.fields.frm = value;
			break;

		// debug CSRs not supported, thus hardwired
		case TSELECT_ADDR:
		case TDATA1_ADDR:
		case TDATA2_ADDR:
		case TDATA3_ADDR:
		case DCSR_ADDR:
		case DPC_ADDR:
		case DSCRATCH0_ADDR:
		case DSCRATCH1_ADDR:
			break;

		default:
			if (!csrs.is_valid_csr32_addr(addr))
				RAISE_ILLEGAL_INSTRUCTION();

			csrs.default_write32(addr, value);
	}
}

void ISS::init(instr_memory_if *instr_mem, data_memory_if *data_mem, clint_if *clint, uint32_t entrypoint,
               uint32_t sp) {
	this->instr_mem = instr_mem;
	this->mem = data_mem;
	this->clint = clint;
	regs[RegFile::sp] = sp;
	pc = entrypoint;
}

void ISS::sys_exit() {
	shall_exit = true;
}

unsigned ISS::get_syscall_register_index() {
	if (csrs.misa.has_E_base_isa())
		return RegFile::a5;
	else
		return RegFile::a7;
}

uint64_t ISS::read_register(unsigned idx) {
	return (uint32_t)regs.read(idx);  // NOTE: zero extend
}

void ISS::write_register(unsigned idx, uint64_t value) {
	// Since the value parameter in the function prototype is
	// a uint64_t, signed integer values (e.g. -1) get promoted
	// to values within this range. For example, -1 would be
	// promoted to (2**64)-1. As such, we cannot perform a
	// Boost lexical or numeric cast to uint32_t here as
	// these perform bounds checks. Instead, we perform a C
	// cast without bounds checks.
	regs.write(idx, (uint32_t)value);
}

uint64_t ISS::get_progam_counter(void) {
	return pc;
}

void ISS::block_on_wfi(bool block) {
	ignore_wfi = !block;
}

CoreExecStatus ISS::get_status(void) {
	return status;
}

void ISS::set_status(CoreExecStatus s) {
	status = s;
}

void ISS::enable_debug(void) {
	debug_mode = true;
}

void ISS::insert_breakpoint(uint64_t addr) {
	breakpoints.insert(addr);
}

void ISS::remove_breakpoint(uint64_t addr) {
	breakpoints.erase(addr);
}

uint64_t ISS::get_hart_id() {
	return csrs.mhartid.reg;
}

std::vector<uint64_t> ISS::get_registers(void) {
	std::vector<uint64_t> regvals;

	for (auto v : regs.regs) regvals.push_back((uint32_t)v);  // NOTE: zero extend

	return regvals;
}

void ISS::fp_finish_instr() {
	fp_set_dirty();
	fp_update_exception_flags();
}

void ISS::fp_prepare_instr() {
	assert(softfloat_exceptionFlags == 0);
	fp_require_not_off();
}

void ISS::fp_set_dirty() {
	csrs.mstatus.fields.sd = 1;
	csrs.mstatus.fields.fs = FS_DIRTY;
}

void ISS::fp_update_exception_flags() {
	if (softfloat_exceptionFlags) {
		fp_set_dirty();
		csrs.fcsr.fields.fflags |= softfloat_exceptionFlags;
		softfloat_exceptionFlags = 0;
	}
}

void ISS::fp_setup_rm() {
	auto rm = instr.frm();
	if (rm == FRM_DYN)
		rm = csrs.fcsr.fields.frm;
	if (rm >= FRM_RMM)
		RAISE_ILLEGAL_INSTRUCTION();
	softfloat_roundingMode = rm;
}

void ISS::fp_require_not_off() {
	if (csrs.mstatus.fields.fs == FS_OFF)
		RAISE_ILLEGAL_INSTRUCTION();
}

void ISS::return_from_trap_handler(PrivilegeLevel return_mode) {
	switch (return_mode) {
		case MachineMode:
			prv = csrs.mstatus.fields.mpp;
			csrs.mstatus.fields.mie = csrs.mstatus.fields.mpie;
			csrs.mstatus.fields.mpie = 1;
			pc = csrs.mepc.reg;
			if (csrs.misa.has_user_mode_extension())
				csrs.mstatus.fields.mpp = UserMode;
			else
				csrs.mstatus.fields.mpp = MachineMode;
			break;

		case SupervisorMode:
			prv = csrs.mstatus.fields.spp;
			csrs.mstatus.fields.sie = csrs.mstatus.fields.spie;
			csrs.mstatus.fields.spie = 1;
			pc = csrs.sepc.reg;
			if (csrs.misa.has_user_mode_extension())
				csrs.mstatus.fields.spp = UserMode;
			else
				csrs.mstatus.fields.spp = SupervisorMode;
			break;

		case UserMode:
			prv = UserMode;
			csrs.mstatus.fields.uie = csrs.mstatus.fields.upie;
			csrs.mstatus.fields.upie = 1;
			pc = csrs.uepc.reg;
			break;

		default:
			throw std::runtime_error("unknown privilege level " + std::to_string(return_mode));
	}

	if (trace)
		printf("[vp::iss] return from trap handler, time %s, pc %8x, prv %1x\n",
		       quantum_keeper.get_current_time().to_string().c_str(), pc, prv);
}

void ISS::trigger_external_interrupt(PrivilegeLevel level) {
	if (trace)
		std::cout << "[vp::iss] trigger external interrupt, " << sc_core::sc_time_stamp() << std::endl;

	switch (level) {
		case UserMode:
			csrs.mip.fields.ueip = true;
			break;
		case SupervisorMode:
			csrs.mip.fields.seip = true;
			break;
		case MachineMode:
			csrs.mip.fields.meip = true;
			break;
	}

	wfi_event.notify(sc_core::SC_ZERO_TIME);
}

void ISS::clear_external_interrupt(PrivilegeLevel level) {
	if (trace)
		std::cout << "[vp::iss] clear external interrupt, " << sc_core::sc_time_stamp() << std::endl;

	switch (level) {
		case UserMode:
			csrs.mip.fields.ueip = false;
			break;
		case SupervisorMode:
			csrs.mip.fields.seip = false;
			break;
		case MachineMode:
			csrs.mip.fields.meip = false;
			break;
	}
}

void ISS::trigger_timer_interrupt(bool status) {
	if (trace)
		std::cout << "[vp::iss] trigger timer interrupt=" << status << ", " << sc_core::sc_time_stamp() << std::endl;
	csrs.mip.fields.mtip = status;
	wfi_event.notify(sc_core::SC_ZERO_TIME);
}

void ISS::trigger_software_interrupt(bool status) {
	if (trace)
		std::cout << "[vp::iss] trigger software interrupt=" << status << ", " << sc_core::sc_time_stamp() << std::endl;
	csrs.mip.fields.msip = status;
	wfi_event.notify(sc_core::SC_ZERO_TIME);
}

PrivilegeLevel ISS::prepare_trap(SimulationTrap &e) {
	// undo any potential pc update (for traps the pc should point to the originating instruction and not it's
	// successor)
	pc = last_pc;
	unsigned exc_bit = (1 << e.reason);

	// 1) machine mode execution takes any traps, independent of delegation setting
	// 2) non-delegated traps are processed in machine mode, independent of current execution mode
	if (prv == MachineMode || !(exc_bit & csrs.medeleg.reg)) {
		csrs.mcause.fields.interrupt = 0;
		csrs.mcause.fields.exception_code = e.reason;
		csrs.mtval.reg = boost::lexical_cast<uint32_t>(e.mtval);
		return MachineMode;
	}

	// see above machine mode comment
	if (prv == SupervisorMode || !(exc_bit & csrs.sedeleg.reg)) {
		csrs.scause.fields.interrupt = 0;
		csrs.scause.fields.exception_code = e.reason;
		csrs.stval.reg = boost::lexical_cast<uint32_t>(e.mtval);
		return SupervisorMode;
	}

	assert(prv == UserMode && (exc_bit & csrs.medeleg.reg) && (exc_bit & csrs.sedeleg.reg));
	csrs.ucause.fields.interrupt = 0;
	csrs.ucause.fields.exception_code = e.reason;
	csrs.utval.reg = boost::lexical_cast<uint32_t>(e.mtval);
	return UserMode;
}

void ISS::prepare_interrupt(const PendingInterrupts &e) {
	if (trace) {
		std::cout << "[vp::iss] prepare interrupt, pending=" << e.pending << ", target-mode=" << e.target_mode
		          << std::endl;
	}

	csr_mip x{e.pending};

	ExceptionCode exc;
	if (x.fields.meip)
		exc = EXC_M_EXTERNAL_INTERRUPT;
	else if (x.fields.msip)
		exc = EXC_M_SOFTWARE_INTERRUPT;
	else if (x.fields.mtip)
		exc = EXC_M_TIMER_INTERRUPT;
	else if (x.fields.seip)
		exc = EXC_S_EXTERNAL_INTERRUPT;
	else if (x.fields.ssip)
		exc = EXC_S_SOFTWARE_INTERRUPT;
	else if (x.fields.stip)
		exc = EXC_S_TIMER_INTERRUPT;
	else if (x.fields.ueip)
		exc = EXC_U_EXTERNAL_INTERRUPT;
	else if (x.fields.usip)
		exc = EXC_U_SOFTWARE_INTERRUPT;
	else if (x.fields.utip)
		exc = EXC_U_TIMER_INTERRUPT;
	else
		throw std::runtime_error("some pending interrupt must be available here");

	switch (e.target_mode) {
		case MachineMode:
			csrs.mcause.fields.exception_code = exc;
			csrs.mcause.fields.interrupt = 1;
			break;

		case SupervisorMode:
			csrs.scause.fields.exception_code = exc;
			csrs.scause.fields.interrupt = 1;
			break;

		case UserMode:
			csrs.ucause.fields.exception_code = exc;
			csrs.ucause.fields.interrupt = 1;
			break;

		default:
			throw std::runtime_error("unknown privilege level " + std::to_string(e.target_mode));
	}
}

PendingInterrupts ISS::compute_pending_interrupts() {
	uint32_t pending = csrs.mie.reg & csrs.mip.reg;

	if (!pending)
		return {NoneMode, 0};

	auto m_pending = pending & ~csrs.mideleg.reg;
	if (m_pending && (prv < MachineMode || (prv == MachineMode && csrs.mstatus.fields.mie))) {
		return {MachineMode, m_pending};
	}

	pending = pending & csrs.mideleg.reg;
	auto s_pending = pending & ~csrs.sideleg.reg;
	if (s_pending && (prv < SupervisorMode || (prv == SupervisorMode && csrs.mstatus.fields.sie))) {
		return {SupervisorMode, s_pending};
	}

	auto u_pending = pending & csrs.sideleg.reg;
	if (u_pending && (prv == UserMode && csrs.mstatus.fields.uie)) {
		return {UserMode, u_pending};
	}

	return {NoneMode, 0};
}

void ISS::switch_to_trap_handler(PrivilegeLevel target_mode) {
	if (trace) {
		printf("[vp::iss] switch to trap handler, time %s, last_pc %8x, pc %8x, irq %u, t-prv %1x\n",
		       quantum_keeper.get_current_time().to_string().c_str(), last_pc, pc, csrs.mcause.fields.interrupt,
		       target_mode);
	}

	// free any potential LR/SC bus lock before processing a trap/interrupt
	release_lr_sc_reservation();

	auto pp = prv;
	prv = target_mode;

	switch (target_mode) {
		case MachineMode:
			csrs.mepc.reg = pc;

			csrs.mstatus.fields.mpie = csrs.mstatus.fields.mie;
			csrs.mstatus.fields.mie = 0;
			csrs.mstatus.fields.mpp = pp;

			pc = csrs.mtvec.get_base_address();

			if (pc == 0) {
				if (error_on_zero_traphandler) {
					throw std::runtime_error("[ISS] Took null trap handler in machine mode");
				} else {
					static bool once = true;
					if (once)
						std::cout
						    << "[ISS] Warn: Taking trap handler in machine mode to 0x0, this is probably an error."
						    << std::endl;
					once = false;
				}
			}

			if (csrs.mcause.fields.interrupt && csrs.mtvec.fields.mode == csr_mtvec::Mode::Vectored)
				pc += 4 * csrs.mcause.fields.exception_code;
			break;

		case SupervisorMode:
			assert(prv == SupervisorMode || prv == UserMode);

			csrs.sepc.reg = pc;

			csrs.mstatus.fields.spie = csrs.mstatus.fields.sie;
			csrs.mstatus.fields.sie = 0;
			csrs.mstatus.fields.spp = pp;

			pc = csrs.stvec.get_base_address();

			if (csrs.scause.fields.interrupt && csrs.stvec.fields.mode == csr_mtvec::Mode::Vectored)
				pc += 4 * csrs.scause.fields.exception_code;
			break;

		case UserMode:
			assert(prv == UserMode);

			csrs.uepc.reg = pc;

			csrs.mstatus.fields.upie = csrs.mstatus.fields.uie;
			csrs.mstatus.fields.uie = 0;

			pc = csrs.utvec.get_base_address();

			if (csrs.ucause.fields.interrupt && csrs.utvec.fields.mode == csr_mtvec::Mode::Vectored)
				pc += 4 * csrs.ucause.fields.exception_code;
			break;

		default:
			throw std::runtime_error("unknown privilege level " + std::to_string(target_mode));
	}
}

void ISS::performance_and_sync_update(Opcode::Mapping executed_op) {
	++total_num_instr;

	if (!csrs.mcountinhibit.fields.IR)
		++csrs.instret.reg;

	if (lr_sc_counter != 0) {
		--lr_sc_counter;
		assert(lr_sc_counter >= 0);
		if (lr_sc_counter == 0)
			release_lr_sc_reservation();
	}

	auto new_cycles = instr_cycles[executed_op];

	if (!csrs.mcountinhibit.fields.CY)
		cycle_counter += new_cycles;

	quantum_keeper.inc(new_cycles);
	if (quantum_keeper.need_sync()) {
		if (lr_sc_counter == 0)  // match SystemC sync with bus unlocking in a tight LR_W/SC_W loop
			quantum_keeper.sync();
	}
}

void ISS::run_step() {
	assert(regs.read(0) == 0);

	// speeds up the execution performance (non debug mode) significantly by
	// checking the additional flag first
	if (debug_mode && (breakpoints.find(pc) != breakpoints.end())) {
		status = CoreExecStatus::HitBreakpoint;
		return;
	}

	last_pc = pc;
	try {
		exec_step();

		auto x = compute_pending_interrupts();
		if (x.target_mode != NoneMode) {
			prepare_interrupt(x);
			switch_to_trap_handler(x.target_mode);
		}
	} catch (SimulationTrap &e) {
		if (trace)
			std::cout << "take trap " << e.reason << ", mtval=" << e.mtval << std::endl;
		auto target_mode = prepare_trap(e);
		switch_to_trap_handler(target_mode);
	}

	// NOTE: writes to zero register are supposedly allowed but must be ignored
	// (reset it after every instruction, instead of checking *rd != zero*
	// before every register write)
	regs.regs[regs.zero] = 0;

	// Do not use a check *pc == last_pc* here. The reason is that due to
	// interrupts *pc* can be set to *last_pc* accidentally (when jumping back
	// to *mepc*).
	if (shall_exit)
		status = CoreExecStatus::Terminated;

	performance_and_sync_update(op);
}

void ISS::run() {
	// run a single step until either a breakpoint is hit or the execution
	// terminates
	do {
		run_step();
	} while (status == CoreExecStatus::Runnable);

	// force sync to make sure that no action is missed
	quantum_keeper.sync();
}

void ISS::show() {
	boost::io::ios_flags_saver ifs(std::cout);
	std::cout << "=[ core : " << csrs.mhartid.reg << " ]===========================" << std::endl;
	std::cout << "simulation time: " << sc_core::sc_time_stamp() << std::endl;
	regs.show();
	std::cout << "pc = " << std::hex << pc << std::endl;
	std::cout << "num-instr = " << std::dec << csrs.instret.reg << std::endl;
}
