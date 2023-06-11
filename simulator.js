const fs       = require('fs');
const readline = require('node:readline');

function createInitialMachineState() {
  const registers = {
    A:   0, /* accumulator               */
    Y:   0, /* index register y          */
    X:   0, /* index register x          */
    PCH: 0, /* program counter high      */
    PCL: 0, /* program counter low       */
    S:   0, /* stack pointer             */
    P:   0  /* processor status register */
  }

  /* Initialize program status register */
  registers.P |= 0b00110110;

  /* Initialze stack pointer */
  registers.S = 0xFF;

  const memory = new Uint8Array(2 ** 16);

  return { registers, memory, nCycles: 0 };
}

function getNegative(state) {
  return (state.registers.P >> 7 & 1) === 1;
}

function setN(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 7;
  } else {
    state.registers.P &= ~(1 << 7);
  }
}

function getOverflow(state) {
  return (state.registers.P >> 6 & 1) === 1;
}

function setV(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 6;
  } else {
    state.registers.P &= ~(1 << 6);
  }
}

function getBrk(state) {
  return (state.registers.P >> 4 & 1) === 1;
}

function setB(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 4;
  } else {
    state.registers.P &= ~(1 << 4);
  }
}

function getDecimal(state) {
  return (state.registers.P >> 3 & 1) === 1;
}

function setD(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 3;
  } else {
    state.registers.P &= ~(1 << 3);
  }
}

function getIRQDisable(state) {
  return (state.registers.P >> 2 & 1) === 1;
}

function setI(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 2;
  } else {
    state.registers.P &= ~(1 << 2);
  }
}

function getZero(state) {
  return (state.registers.P >> 1 & 1) === 1;
}

function setZ(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 1;
  } else {
    state.registers.P &= ~(1 << 1);
  }
}

function getC(state) {
  return (state.registers.P & 1) === 1;
}

function setC(state, bit) {
  if(bit) {
    state.registers.P |= 1;
  } else {
    state.registers.P &= ~1;
  }
}

function asAccum(_address, _state) {
  /* With Accumulator addressing the operand is implied as
   * the Accumulator and therefore only a single byte
   * forms the instruction. */
  return {
    nBytes: 1
  }
}

function asIMM(address, state) {
  /* With Immediate Addressing the operand is the second byte
   * of the instruction. */
  const operand = state.memory[address + 1];
  return {
    nBytes: 2,
    Operand: operand
  }
}

function asABS(address, state) {
  /* With Absolute addressing the second and third bytes of
   * the instruction form the 16-bit address */
  const adl = state.memory[address + 1];
  const adh = state.memory[address + 2];
  let operandAddress = 0; /* 16b */
  operandAddress  = adl;
  operandAddress += adh << 8;
  return {
    nBytes: 3,
    OperandAddress: operandAddress
  }
}

function asZP(address, state) {
  /* With Zero Page (zp) addressing the second byte of the
   * instruction is the address of the operand in page
   * zero. */
  const zp = state.memory[address + 1];
  const operandAddress = zp;
  return {
    nBytes: 2,
    OperandAddress: operandAddress
  }
}

function asZPX(address, state) {
  /* With Zero Page Indexed with X addressing mode, the X
   * Index Register is added to the second byte of
   * instruction to form the effective address. */
  const zp = state.memory[address + 1];
  const operandAddress = asByte(zp + state.registers.X);
  return {
    nBytes: 2,
    OperandAddress: operandAddress
  }
}

function asZPY(address, state) {
  /* With Zero Page Indexed with Y addressing, the second
   * byte of the instruction is the zero page address to
   * which the Y Index Register is added to form the page
   * zero effective address. */
  const zp = state.memory[address + 1];
  const operandAddress = asByte(zp + state.registers.Y);
  return {
    nBytes: 2,
    OperandAddress: operandAddress
  }
}

function asABSX(address, state) {
  /* With the Absolute Indexed with X addressing mode, the X
   * Index Register is added to the second and third bytes of
   * the instruction to form the 16-bits of the effective address. */
  const adl = state.memory[address + 1];
  const adh = state.memory[address + 2];
  let operandAddress = 0; /* 16b */
  operandAddress  = adl;
  operandAddress += adh << 8;
  const effectiveAddress = asWord(operandAddress + state.registers.X);
  return {
    nBytes: 3,
    OperandAddress: effectiveAddress
  }
}

function asABSY(address, state) {
  /* With the Absolute Indexed with Y addressing mode, the Y
   * Index Register is added to the second and third bytes of
   * the instruction to form the 16-bits of the effective address. */
  const adl = state.memory[address + 1];
  const adh = state.memory[address + 2];
  let operandAddress = 0; /* 16b */
  operandAddress  = adl;
  operandAddress += adh << 8;
  const effectiveAddress = asWord(operandAddress + state.registers.Y);
  return {
    nBytes: 3,
    OperandAddress: effectiveAddress
  }
}

function asImplied(_address, _state) {
  /* Implied addressing uses a single byte instruction.
   * The operand is implicitly defined by the instruction. */
  return {
    nBytes: 1
  }
}

function asRelative(address, state) {
  /* The Program Counter relative addressing mode, sometimes
   * referred to as Relative Addressing, is used with the
   * Branch instructions. If the condition being tested is
   * met, the second byte of the instruction is added to the
   * Program Counter and program control is transferred to
   * this new memory location. */
  let offset = state.memory[address + 1];
  if(offset >= 128) offset -= 256;

  let pc = state.registers.PCL;
  pc += state.registers.PCH << 8;
  const effectiveAddress = asWord(pc + offset);
  return {
    nBytes: 2,
    NewPCValue: effectiveAddress + 2
  }
}

function asINDX(address, state) {
  /* The Zero Page Indexed Indirect addressing mode is often
   * referred to as Indirect,X. The second byte of the
   * instruction is the zero page address to which the X Index
   * Register is added and the result points to the low byte of
   * the indirect address. */
  const zp = state.memory[address + 1];
  const indirectAddress = asByte(zp + state.registers.X);
  let operandAddress = state.memory[indirectAddress];
  operandAddress += state.memory[indirectAddress + 1] << 8;
  return {
    nBytes: 2,
    OperandAddress: operandAddress
  }
}

function asINDY(address, state) {
  /* The Zero Page Indirect Indexed with Y addressing mode
   * is often referred to as Indirect Y. The second byte of
   * the instruction points to the low byte of a two byte
   * (16-bit) base address in page zero. Y Index Register is
   * added to the base address to form the effective address. */
  const zp = state.memory[address + 1];
  let indirectBaseAddress = state.memory[zp];
  indirectBaseAddress += state.memory[zp + 1] << 8;
  const operandAddress = asWord(indirectBaseAddress + state.registers.Y);
  return {
    nBytes: 2,
    OperandAddress: operandAddress
  }
}

function asIndirect(address, state) {
  /* With the Absolute Indirect addressing mode, the second
   * and third bytes of the instruction form an address
   * to a pointer. This address mode is only used with the JMP
   * instruction and the Program Counter is loaded
   * with the first and second bytes at this pointer */
  const ial = state.memory[address + 1];
  const iah = state.memory[address + 2];
  let indirectAddress = ial;
  indirectAddress += iah << 8;
  const idl = state.memory[indirectAddress];
  const idh = state.memory[asWord(indirectAddress + 1)];
  let newpc = idl;
  newpc += idh << 8;
  return {
    nBytes: 3,
    NewPCValue: newpc
  }
}

function updateN(state, number) {
  setN(state, (number >> 7 & 1) === 1);
}

function updateZ(state, number) {
  setZ(state, number === 0);
}

function updateC(state, number) {
  setC(state, (number & ~0xFF) !== 0);
}

function updateBorrow(state, number) {
  updateC(state, number);
  setC(state, !getC(state));
}

function updateV(state, number) {
  const bit6 = number >> 6 & 1;
  const bit7 = number >> 7 & 1;
  setV(state, (bit6 ^ bit7) === 1);
}

function asByte(number) {
  return number & 0xFF;
}

function asWord(number) {
  return number & 0xFFFF;
}

function stepPC(state, bytes) {
  let pc = (state.registers.PCH << 8) + state.registers.PCL;
  pc = asWord(pc + bytes);
  setPC(state, pc);
}

function setPC(state, newPC) {
  state.registers.PCL = newPC & 0xFF;
  state.registers.PCH = (newPC & 0xFF00) >> 8;
}

function pcAsInteger(state) {
  let pc = state.registers.PCL;
  pc += state.registers.PCH << 8;
  return pc;
}

function incrementCycles(state, nCycles) {
  state.nCycles += nCycles;
}

function opADC(state, addr) {
  const A = state.registers.A;
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  const C = getC(state) ? 1 : 0;
  let sum = A + M + C;
  updateC(state, sum);
  sum = asByte(sum);
  updateN(state, sum);
  updateZ(state, sum);
  updateV(state, sum);
  state.registers.A = sum;
  stepPC(state, addr.nBytes);
}

function opAND(state, addr) {
  const A = state.registers.A;
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  const and = A & M;
  updateN(state, and);
  updateZ(state, end);
  state.registers.A = and;
  stepPC(state, addr.nBytes);
}

function opASL(state, addr) {
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : state.registers.A;
  let lsl = M << 1;
  updateC(state, lsl);
  lsl = asByte(lsl);
  updateN(state, lsl);
  updateZ(state, lsl);
  state.registers.A = lsl;
  stepPC(state, addr.nBytes);
}

function opBCC(state, addr) {
  if(!getC(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBCS(state, addr) {
  if(getC(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBEQ(state, addr) {
  if(getZero(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBIT(state, addr) {
  const A = state.registers.A;
  const M = state.memory[addr.OperandAddress];
  let and = A & M;
  updateZ(state, and);
  setN(state, (M >> 7 & 1) === 1);
  setV(state, (M >> 6 & 1) === 1);
  stepPC(state, addr.nBytes);
}

function opBMI(state, addr) {
  if(getNegative(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBNE(state, addr) {
  if(!getZero(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBPL(state, addr) {
  if(!getNegative(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBRK(state, _addr) {
  const pcPlusTwo = asWord(pcAsInteger(state) + 2);
  const PCL = pcPlusTwo & 0x00FF;
  const PCH = (pcPlusTwo & 0xFF00) >> 8;
  state.memory[0x0100 | state.registers.S] = PCH;
  state.registers.S = asByte(state.registers.S - 1);
  state.memory[0x0100 | state.registers.S] = PCL;
  state.registers.S = asByte(state.registers.S - 1);
  state.memory[0x0100 | state.registers.S] = state.registers.P;
  state.registers.S = asByte(state.registers.S - 1);
  state.registers.PCL = state.memory[0xFFFE];
  state.registers.PCH = state.memory[0xFFFF];
  setB(state, true);
  setI(state, true);
}

function opBVC(state, addr) {
  if(!getOverflow(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBVS(state, addr) {
  if(getOverflow(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opCLC(state, addr) {
  setC(state, false);
  stepPC(state, addr.nBytes);
}

function opCLD(state, addr) {
  setD(state, false);
  stepPC(state, addr.nBytes);
}

function opCLI(state, addr) {
  setI(state, false);
  stepPC(state, addr.nBytes);
}

function opCLV(state, addr) {
  setV(state, false);
  stepPC(state, addr.nBytes);
}

function opCMP(state, addr) {
  const A = state.registers.A;
  // set borrow = 0
  setC(state, true);
  SBC(A, state, addr);
  stepPC(state, addr.nBytes);
}

function opCPX(state, addr) {
  const X = state.registers.X;
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  setN(state, (asByte(X - M) >> 7) === 1);
  setZ(state, X === M);
  setC(state, X >= M);
  stepPC(state, addr.nBytes);
}

function opCPY(state, addr) {
  const Y = state.registers.Y;
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  setN(state, (asByte(Y - M) >> 7) === 1);
  setZ(state, M === Y);
  setC(state, Y >= M);
  stepPC(state, addr.nBytes);
}

function opDEC(state, addr) {
  const result = state.memory[addr.OperandAddress] =
    asByte(state.memory[addr.OperandAddress] - 1);
  updateN(state, result);
  updateZ(state, result);
  stepPC(state, addr.nBytes);
}

function opDEX(state, addr) {
  state.registers.X = asByte(state.registers.X - 1);
  updateN(state, state.registers.X);
  updateZ(state, state.registers.X);
  stepPC(state, addr.nBytes);
}

function opDEY(state, addr) {
  state.registers.Y = asByte(state.registers.Y - 1);
  updateN(state, state.registers.Y);
  updateZ(state, state.registers.Y);
  stepPC(state, addr.nBytes);
}

function opEOR(state, addr) {
  const A = state.registers.A;
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  const eor = A ^ M;
  updateN(state, eor);
  updateZ(state, eor);
  state.registers.A = eor;
  stepPC(state, addr.nBytes);
}

function opINC(state, addr) {
  const result = state.memory[addr.OperandAddress] =
    asByte(state.memory[addr.OperandAddress] + 1);
  updateN(state, result);
  updateZ(state, result);
  stepPC(state, addr.nBytes);
}

function opINX(state, addr) {
  state.registers.X = asByte(state.registers.X + 1);
  updateN(state, state.registers.X);
  updateZ(state, state.registers.X);
  stepPC(state, addr.nBytes);
}

function opINY(state, addr) {
  state.registers.Y = asByte(state.registers.Y + 1);
  updateN(state, state.registers.Y);
  updateZ(state, state.registers.Y);
  stepPC(state, addr.nBytes);
}

function opJMP(state, addr) {
  const pc = addr.NewPCValue !== undefined
           ? addr.NewPCValue
           : addr.OperandAddress;
  setPC(state, pc);
}

function opJSR(state, addr) {
  stepPC(state, addr.nBytes);
  const pcMinusOne = asWord(pcAsInteger(state) - 1);
  const PCL = pcMinusOne & 0x00FF;
  const PCH = (pcMinusOne & 0xFF00) >> 8;
  state.memory[0x0100 | state.registers.S] = PCH;
  state.registers.S = asByte(state.registers.S - 1);
  state.memory[0x0100 | state.registers.S] = PCL;
  state.registers.S = asByte(state.registers.S - 1);
  setPC(state, addr.OperandAddress);
}

function opLDA(state, addr) {
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  state.registers.A = M;
  updateN(state, M);
  updateZ(state, M);
  stepPC(state, addr.nBytes);
}

function opLDX(state, addr) {
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  state.registers.X = M;
  updateN(state, M);
  updateZ(state, M);
  stepPC(state, addr.nBytes);
}

function opLDY(state, addr) {
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  state.registers.Y = M;
  updateN(state, M);
  updateZ(state, M);
  stepPC(state, addr.nBytes);
}

function opLSR(state, addr) {
  let M = addr.OperandAddress !== undefined
        ? state.memory[addr.OperandAddress]
        : state.registers.A;
  setC(state, (M & 1) === 1);
  M = M >> 1 & 0b01111111;
  if(addr.OperandAddress !== undefined) {
    state.memory[addr.OperandAddress] = M;
  } else {
    state.registers.A = M;
  }
  setN(state, false);
  updateZ(state, M);
  stepPC(state, addr.nBytes);
}

function opNOP(state, addr) {
  stepPC(state, addr.nBytes);
}

function opORA(state, addr) {
  const A = state.registers.A;
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  const or = A | M;
  updateN(state, or);
  updateZ(state, or);
  state.registers.A = or;
  stepPC(state, addr.nBytes);
}

function opPHA(state, addr) {
  state.memory[0x0100 | state.registers.S] = state.registers.A;
  state.registers.S = asByte(state.registers.S - 1);
  stepPC(state, addr.nBytes);
}

function opPHP(state, addr) {
  state.memory[0x0100 | state.registers.S] = state.registers.P;
  state.registers.S = asByte(state.registers.S - 1);
  stepPC(state, addr.nBytes);
  setB(state, true);
}

function opPLA(state, addr) {
  state.registers.S = asByte(state.registers.S + 1);
  state.registers.A = state.memory[0x0100 | state.registers.S];
  updateN(state, state.registers.A);
  updateZ(state, state.registers.A);
  stepPC(state, addr.nBytes);
}

function opPLP(state, addr) {
  state.registers.S = asByte(state.registers.S + 1);
  const status = state.memory[0x0100 | state.registers.S];
  state.registers.P &= 0b00110000;
  state.registers.P |= status & 0b11001111;
  stepPC(state, addr.nBytes);
}

function opROL(state, addr) {
  let M = addr.OperandAddress !== undefined
        ? state.memory[addr.OperandAddress]
        : state.registers.A;
  const msb = (M >> 7) === 1;
  setN(state, (M >> 6 & 1) === 1);
  M = asByte(M << 1 & 0b11111110 | (getC(state) ? 1 : 0));
  if(addr.OperandAddress !== undefined) {
    state.memory[addr.OperandAddress] = M;
  } else {
    state.registers.A = M;
  }
  setC(state, msb);
  updateZ(state, M);
  stepPC(state, addr.nBytes);
}

function opROR(state, addr) {
  let M = addr.OperandAddress !== undefined
        ? state.memory[addr.OperandAddress]
        : state.registers.A;
  const lsb = (M & 1) === 1;
  setN(state, getC(state));
  M = asByte(M >> 1 & 0x01111111 | (getC(state) ? 1 : 0) << 7);
  if(addr.OperandAddress !== undefined) {
    state.memory[addr.OperandAddress] = M;
  } else {
    state.registers.A = M;
  }
  setC(state, lsb);
  updateZ(state, M);
  stepPC(state, addr.nBytes);
}

function opRTI(state, _addr) {
  state.registers.S = asByte(state.registers.S + 1);
  const status = state.memory[0x0100 | state.registers.S];
  state.registers.P &= 0b00110000;
  state.registers.P |= status & 0b11001111;

  state.registers.S = asByte(state.registers.S + 1);
  state.registers.PCL = state.memory[0x0100 | state.registers.S];
  state.registers.S = asByte(state.registers.S + 1);
  state.registers.PCH = state.memory[0x0100 | state.registers.S];
}

function opRTS(state, _addr) {
  state.registers.S = asByte(state.registers.S + 1);
  state.registers.PCL = state.memory[0x0100 | state.registers.S];
  state.registers.S = asByte(state.registers.S + 1);
  state.registers.PCH = state.memory[0x0100 | state.registers.S];
  stepPC(state, 1);
}

/* do the SBC operation taking the LHS byte as a parameter,
 * and don't write out the result. Program counter is
 * not incremented, but flags are updated. */
function SBC(lhs, state, addr) {
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  const C = getC(state) ? 1 : 0;
  let sum = lhs + (~M) + C;
  updateBorrow(state, sum);
  sum = asByte(sum);
  updateN(state, sum);
  updateZ(state, sum);
  return sum;
}

function opSBC(state, addr) {
  const A = state.registers.A;
  state.registers.A = SBC(A, state, addr);
  updateV(state, sum);
  stepPC(state, addr.nBytes);
}

function opSEC(state, addr) {
  setC(state, true);
  stepPC(state, addr.nBytes);
}

function opSED(state, addr) {
  setD(state, true);
  stepPC(state, addr.nBytes);
}

function opSEI(state, addr) {
  setI(state, true);
  stepPC(state, addr.nBytes);
}

function opSTA(state, addr) {
  state.memory[addr.OperandAddress] = state.registers.A;
  stepPC(state, addr.nBytes);
}

function opSTX(state, addr) {
  state.memory[addr.OperandAddress] = state.registers.X;
  stepPC(state, addr.nBytes);
}

function opSTY(state, addr) {
  state.memory[addr.OperandAddress] = state.registers.Y;
  stepPC(state, addr.nBytes);
}

function opTAX(state, addr) {
  state.registers.X = state.registers.A;
  updateN(state, state.registers.X);
  updateZ(state, state.registers.X);
  stepPC(state, addr.nBytes);
}

function opTAY(state, addr) {
  state.registers.Y = state.registers.A;
  updateN(state, state.registers.Y);
  updateZ(state, state.registers.Y);
  stepPC(state, addr.nBytes);
}

function opTSX(state, addr) {
  state.registers.X = state.registers.S;
  updateN(state, state.registers.X);
  updateZ(state, state.registers.X);

  stepPC(state, addr.nBytes);
}

function opTXA(state, addr) {
  state.registers.A = state.registers.X;
  updateN(state, state.registers.A);
  updateZ(state, state.registers.A);

  stepPC(state, addr.nBytes);
}

function opTXS(state, addr) {
  state.registers.S = state.registers.X;

  stepPC(state, addr.nBytes);
}

function opTYA(state, addr) {
  state.registers.A = state.registers.Y;
  updateN(state, state.registers.A);
  updateZ(state, state.registers.A);

  stepPC(state, addr.nBytes);
}

const opDecodeMatrix = [
  [ /* 0 (MSD) */
    /* 0 (LSD) */ [opBRK, asImplied, "BRK Implied"],
    /* 1       */ [opORA, asINDX,    "ORA INDX"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opORA, asZP,      "ORA ZP"],
    /* 6       */ [opASL, asZP,      "ASL ZP"],
    /* 7       */ null,
    /* 8       */ [opPHP, asImplied, "PHP Implied"],
    /* 9       */ [opORA, asIMM,     "ORA IMM"],
    /* A       */ [opASL, asAccum,   "ASL Accum"],
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opORA, asABS,     "ORA ABS"],
    /* E       */ [opASL, asABS,     "ASL ABS"],
    /* F       */ null
  ],
  [ /* 1 */
    /* 0 (LSD) */ [opBPL, asRelative, "BPL Relative"],
    /* 1       */ [opORA, asINDY,     "ORA INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opORA, asZPX,      "ORA ZPX"],
    /* 6       */ [opASL, asZPX,      "ASL ZPX"],
    /* 7       */ null,
    /* 8       */ [opCLC, asImplied,  "CLC Implied"],
    /* 9       */ [opORA, asABSY,     "ORA ABSY"],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opORA, asABSX,     "ORA ABSX"],
    /* E       */ [opASL, asABSX,     "ASL ABSX"],
    /* F       */ null
  ],
  [ /* 2 */
    /* 0 (LSD) */ [opJSR, asABS,     "JSR ABS"],
    /* 1       */ [opAND, asINDX,    "AND INDX"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opBIT, asZP,      "BIT ZP"],
    /* 5       */ [opAND, asZP,      "AND ZP"],
    /* 6       */ [opROL, asZP,      "ROL ZP"],
    /* 7       */ null,
    /* 8       */ [opPLP, asImplied, "PLP Implied"],
    /* 9       */ [opAND, asIMM,     "AND IMM"],
    /* A       */ [opROL, asAccum,   "ROL Accum"],
    /* B       */ null,
    /* C       */ [opBIT, asABS,     "BIT ABS"],
    /* D       */ [opAND, asABS,     "AND ABS"],
    /* E       */ [opROL, asABS,     "ROL ABS"],
    /* F       */ null
  ],
  [ /* 3 */
    /* 0 (LSD) */ [opBMI, asRelative, "BMI Relative"],
    /* 1       */ [opAND, asINDY,     "AND INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opAND, asZPX,      "AND ZPX"],
    /* 6       */ [opROL, asZPX,      "ROL ZPX"],
    /* 7       */ null,
    /* 8       */ [opSEC, asImplied,  "SEC Implied"],
    /* 9       */ [opAND, asABSY,     "AND ABSY"],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opAND, asABSX,     "AND ABSX"],
    /* E       */ [opROL, asABSX,     "ROL ABSX"],
    /* F       */ null
  ],
  [ /* 4 */
    /* 0 (LSD) */ [opRTI, asImplied, "RTI Implied"],
    /* 1       */ [opEOR, asINDX,    "EOR INDX"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opEOR, asZP,      "EOR ZP"],
    /* 6       */ [opLSR, asZP,      "LSR ZP"],
    /* 7       */ null,
    /* 8       */ [opPHA, asImplied, "PHA Implied"],
    /* 9       */ [opEOR, asIMM,     "EOR IMM"],
    /* A       */ [opLSR, asAccum,   "LSR Accum"],
    /* B       */ null,
    /* C       */ [opJMP, asABS,     "JMP ABS"],
    /* D       */ [opEOR, asABS,     "EOR ABS"],
    /* E       */ [opLSR, asABS,     "LSR ABS"],
    /* F       */ null
  ],
  [ /* 5 */
    /* 0 (LSD) */ [opBVC, asRelative, "BVC Relative"],
    /* 1       */ [opEOR, asINDY,     "EOR INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opEOR, asZPX,      "EOR ZPX"],
    /* 6       */ [opLSR, asZPX,      "LSR ZPX"],
    /* 7       */ null,
    /* 8       */ [opCLI, asImplied,  "CLI Implied"],
    /* 9       */ [opEOR, asABSY,     "EOR ABSY"],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opEOR, asABSX,     "EOR ABSX"],
    /* E       */ [opLSR, asABSX,     "LSR ABSX"],
    /* F       */ null
  ],
  [ /* 6 */
    /* 0 (LSD) */ [opRTS, asImplied,  "RTS Implied"],
    /* 1       */ [opADC, asINDX,     "ADC INDX"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opADC, asZP,       "ADC ZP"],
    /* 6       */ [opROR, asZP,       "ROR ZP"],
    /* 7       */ null,
    /* 8       */ [opPLA, asImplied,  "PLA Implied"],
    /* 9       */ [opADC, asIMM,      "ADC IMM"],
    /* A       */ [opROR, asAccum,    "ROR Accum"],
    /* B       */ null,
    /* C       */ [opJMP, asIndirect, "JMP Indirect"],
    /* D       */ [opADC, asABS,      "ADC ABS"],
    /* E       */ [opROR, asABS,      "ROR ABS"],
    /* F       */ null
  ],
  [ /* 7 */
    /* 0 (LSD) */ [opBVS, asRelative, "BVS Relative"],
    /* 1       */ [opADC, asINDY,     "ADC INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opADC, asZPX,      "ADC ZPX"],
    /* 6       */ [opROR, asZPX,      "ROR ZPX"],
    /* 7       */ null,
    /* 8       */ [opSEI, asImplied,  "SEI Implied"],
    /* 9       */ [opADC, asABSY,     "ADC ABSY"],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opADC, asABSX,     "ADC ABSX"],
    /* E       */ [opROR, asABSX,     "ROR ABSX"],
    /* F       */ null
  ],
  [ /* 8 */
    /* 0 (LSD) */ null,
    /* 1       */ [opSTA, asINDX,    "STA INDX"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opSTY, asZP,      "STY ZP"],
    /* 5       */ [opSTA, asZP,      "STA ZP"],
    /* 6       */ [opSTX, asZP,      "STX ZP"],
    /* 7       */ null,
    /* 8       */ [opDEY, asImplied, "DEY Implied"],
    /* 9       */ null,
    /* A       */ [opTXA, asImplied, "TXA Implied"],
    /* B       */ null,
    /* C       */ [opSTY, asABS,     "STY ABS"],
    /* D       */ [opSTA, asABS,     "STA ABS"],
    /* E       */ [opSTX, asABS,     "STX ABS"],
    /* F       */ null
  ],
  [ /* 9 */
    /* 0 (LSD) */ [opBCC, asRelative, "BCC Relative"],
    /* 1       */ [opSTA, asINDY,     "STA INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opSTY, asZPX,      "STY ZPX"],
    /* 5       */ [opSTA, asZPX,      "STA ZPX"],
    /* 6       */ [opSTX, asZPY,      "STX ZPY"],
    /* 7       */ null,
    /* 8       */ [opTYA, asImplied,  "TYA Implied"],
    /* 9       */ [opSTA, asABSY,     "STA ABSY"],
    /* A       */ [opTXS, asImplied,  "TXS Implied"],
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opSTA, asABSX,     "STA ABSX"],
    /* E       */ null,
    /* F       */ null
  ],
  [ /* A */
    /* 0 (LSD) */ [opLDY, asIMM,     "LDY IMM"],
    /* 1       */ [opLDA, asINDX,    "LDA INDX"],
    /* 2       */ [opLDX, asIMM,     "LDX IMM"],
    /* 3       */ null,
    /* 4       */ [opLDY, asZP,      "LDY ZP"],
    /* 5       */ [opLDA, asZP,      "LDA ZP"],
    /* 6       */ [opLDX, asZP,      "LDX ZP"],
    /* 7       */ null,
    /* 8       */ [opTAY, asImplied, "TAY Implied"],
    /* 9       */ [opLDA, asIMM,     "LDA IMM"],
    /* A       */ [opTAX, asImplied, "TAX Implied"],
    /* B       */ null,
    /* C       */ [opLDY, asABS,     "LDY ABS"],
    /* D       */ [opLDA, asABS,     "LDA ABS"],
    /* E       */ [opLDX, asABS,     "LDX ABS"],
    /* F       */ null
  ],
  [ /* B */
    /* 0 (LSD) */ [opBCS, asRelative, "BCS Relative"],
    /* 1       */ [opLDA, asINDY,     "LDA INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opLDY, asZPX,      "LDY ZPX"],
    /* 5       */ [opLDA, asZPX,      "LDA ZPX"],
    /* 6       */ [opLDX, asZPY,      "LDX ZPY"],
    /* 7       */ null,
    /* 8       */ [opCLV, asImplied,  "CLV Implied"],
    /* 9       */ [opLDA, asABSY,     "LDA ABSY"],
    /* A       */ [opTSX, asImplied,  "TSX Implied"],
    /* B       */ null,
    /* C       */ [opLDY, asABSX,     "LDY ABSX"],
    /* D       */ [opLDA, asABSX,     "LDA ABSX"],
    /* E       */ [opLDX, asABSY,     "LDX ABSY"],
    /* F       */ null
  ],
  [ /* C */
    /* 0 (LSD) */ [opCPY, asIMM,     "CPY IMM"],
    /* 1       */ [opCMP, asINDX,    "CMP INDX"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opCPY, asZP,      "CPY ZP"],
    /* 5       */ [opCMP, asZP,      "CMP ZP"],
    /* 6       */ [opDEC, asZP,      "DEC ZP"],
    /* 7       */ null,
    /* 8       */ [opINY, asImplied, "INY Implied"],
    /* 9       */ [opCMP, asIMM,     "CMP IMM"],
    /* A       */ [opDEX, asImplied, "DEX Implied"],
    /* B       */ null,
    /* C       */ [opCPY, asABS,     "CPY ABS"],
    /* D       */ [opCMP, asABS,     "CMP ABS"],
    /* E       */ [opDEC, asABS,     "DEC ABS"],
    /* F       */ null
  ],
  [ /* D */
    /* 0 (LSD) */ [opBNE, asRelative, "BNE Relative"],
    /* 1       */ [opCMP, asINDY,     "CMP INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opCMP, asZPX,      "CMP ZPX"],
    /* 6       */ [opDEC, asZPX,      "DEC ZPX"],
    /* 7       */ null,
    /* 8       */ [opCLD, asImplied,  "CLD Implied"],
    /* 9       */ [opCMP, asABSY,     "CMP ABSY"],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opCMP, asABSX,     "CMP ABSX"],
    /* E       */ [opDEC, asABSX,     "DEC ABSX"],
    /* F       */ null
  ],
  [ /* E */
    /* 0 (LSD) */ [opCPX, asIMM,     "CPX IMM"],
    /* 1       */ [opSBC, asINDX,    "SBC INDX"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opCPX, asZP,      "CPX ZP"],
    /* 5       */ [opSBC, asZP,      "SBC ZP"],
    /* 6       */ [opINC, asZP,      "INC ZP"],
    /* 7       */ null,
    /* 8       */ [opINX, asImplied, "INX Implied"],
    /* 9       */ [opSBC, asIMM,     "SBC IMM"],
    /* A       */ [opNOP, asImplied, "NOP Implied"],
    /* B       */ null,
    /* C       */ [opCPX, asABS,     "CPX ABS"],
    /* D       */ [opSBC, asABS,     "SBC ABS"],
    /* E       */ [opINC, asABS,     "INC ABS"],
    /* F       */ null
  ],
  [ /* F */
    /* 0 (LSD) */ [opBEQ, asRelative, "BEQ Relative"],
    /* 1       */ [opSBC, asINDY,     "SBC INDY"],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opSBC, asZPX,      "SBC ZPX"],
    /* 6       */ [opINC, asZPX,      "INC ZPX"],
    /* 7       */ null,
    /* 8       */ [opSED, asImplied,  "SED Implied"],
    /* 9       */ [opSBC, asABSY,     "SBC ABSY"],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opSBC, asABSX,     "SBC ABSX"],
    /* E       */ [opINC, asABSX,     "INC ABSX"],
    /* F       */ null
  ]
];

function dumpState(state) {
  const sp = () => process.stdout.write(" ");
  process.stdout.write("PC   A  X  Y  S  [N V - B D I Z C]\n");
  process.stdout.write(pcAsInteger(state).toString(16).padStart(4, '0'));
  sp();
  process.stdout.write(state.registers.A.toString(16).padStart(2, '0'));
  sp();
  process.stdout.write(state.registers.X.toString(16).padStart(2, '0'));
  sp();
  process.stdout.write(state.registers.Y.toString(16).padStart(2, '0'));
  sp();
  process.stdout.write(state.registers.S.toString(16).padStart(2, '0'));
  sp(); sp();
  const bToS = (b) => b ? "1" : "0";
  process.stdout.write(bToS(getNegative(state)));
  sp();
  process.stdout.write(bToS(getOverflow(state)));
  sp();
  process.stdout.write("1");
  sp();
  process.stdout.write(bToS(getBrk(state)));
  sp();
  process.stdout.write(bToS(getDecimal(state)));
  sp();
  process.stdout.write(bToS(getIRQDisable(state)));
  sp();
  process.stdout.write(bToS(getZero(state)));
  sp();
  process.stdout.write(bToS(getC(state)));
  process.stdout.write("\n");
}

if(process.argv.length <= 2) {
  process.exit(1);
}

const fname = process.argv[2];
const bin = fs.readFileSync(fname);

const state = createInitialMachineState();

if(bin.length > state.memory.length) {
  process.stderr.write(
    `binary file is too large to fit in memory. (${bin.length})\n`
  );
  process.exit(1);
}

const region = new Uint8Array(
  bin.buffer,
  bin.byteOffset,
  bin.byteLength);

state.memory.set(region, 0);

state.registers.PCH = 0;
state.registers.PCL = 0x0400;

dumpState(state);

const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout,
  prompt: '>>> '
});

rl.prompt();

function step() {
  const pc = pcAsInteger(state);
  const instruction = state.memory[pc];
  const msd = (instruction & 0xF0) >> 4;
  const lsd = instruction & 0x0F;
  const [op, addrMode, description] = opDecodeMatrix[msd][lsd];
  const addr = addrMode(pc, state);
  process.stdout.write("decoded " + description + "\n");
  op(state, addr);
  dumpState(state);
}

rl.on('line', (cmd) => {
  const tokens = cmd.split(' ');

  let didParse = false;
  if(tokens.length === 1) {
    if(cmd === 'q') {
      didParse = true;
      rl.close();
    } else if(cmd === 's') {
      didParse = true;
      step();
    } else if(cmd === 'l') {
      didParse = true;
      let lastPC = null;
      while(lastPC !== pcAsInteger(state)) {
        lastPC = pcAsInteger(state);
        step();
      }
    }
  } else if(tokens.length === 2) {
    if(tokens[0] === 's' && !isNaN(parseInt(tokens[1]))) {
      didParse = true;
      for(let i = 0; i < parseInt(tokens[1]); ++i) {
        step();
      }
    } else if(tokens[0] === 'l' && !isNaN(parseInt(tokens[1], 16))) {
      didParse = true;
      const stopPC = parseInt(tokens[1], 16);
      while(stopPC !== pcAsInteger(state)) {
        step();
      }
    } else if(tokens[0] === 'peek' && !isNaN(parseInt(tokens[1], 16))) {
      didParse = true;
      const addr = parseInt(tokens[1], 16);
      process.stdout.write(state.memory[addr].toString(16) + "\n");
    }
  }

      if(!didParse) {
    process.stdout.write(`unknown command "${cmd}"\n`);
  }

  rl.prompt();
}).on('close', () => {
  process.stdout.write("Goodbye.\n");
  process.exit(0);
});
