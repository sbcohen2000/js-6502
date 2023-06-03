const fs       = require('fs');
const readline = require('node:readline');

function createInitialMachineState() {
  const registerFile = new Uint8Array(7);
  const registers = {
    A:   registerFile[0], /* accumulator               */
    Y:   registerFile[1], /* index register y          */
    X:   registerFile[2], /* index register x          */
    PCH: registerFile[3], /* program counter high      */
    PCL: registerFile[4], /* program counter low       */
    S:   registerFile[5], /* stack pointer             */
    P:   registerFile[6]  /* processor status register */
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

function setNegative(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 7;
  } else {
    state.registers.P &= ~(1 << 7);
  }
}

function getOverflow(state) {
  return (state.registers.P >> 6 & 1) === 1;
}

function setOverflow(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 6;
  } else {
    state.registers.P &= ~(1 << 6);
  }
}

function getBrk(state) {
  return (state.registers.P >> 4 & 1) === 1;
}

function setBrk(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 4;
  } else {
    state.registers.P &= ~(1 << 4);
  }
}

function getDecimal(state) {
  return (state.registers.P >> 3 & 1) === 1;
}

function setDecimal(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 3;
  } else {
    state.registers.P &= ~(1 << 3);
  }
}

function getIRQDisable(state) {
  return (state.registers.P >> 2 & 1) === 1;
}

function setIRQDisable(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 2;
  } else {
    state.registers.P &= ~(1 << 2);
  }
}

function getZero(state) {
  return (state.registers.P >> 1 & 1) === 1;
}

function setZero(state, bit) {
  if(bit) {
    state.registers.P |= 1 << 1;
  } else {
    state.registers.P &= ~(1 << 1);
  }
}

function getCarry(state) {
  return (state.registers.P & 1) === 1;
}

function setCarry(state, bit) {
  if(bit) {
    state.registers.P |= 1;
  } else {
    state.registers.P &= ~1;
  }
}

function getBorrow(state) {
  return !getCarry(state);
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
  const operandAddress = state.memory[indirectAddress];
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
  const adl = state.memory[address + 1];
  const adh = state.memory[address + 2];
  let indirectAddress = 0; /* 16b */
  indirectAddress  = adl;
  indirectAddress += adh << 8;
  return {
    nBytes: 3,
    NewPCValue: indirectAddress
  }
}

function updateN(state, number) {
  setNegative(state, (number >> 7 & 1) === 1);
}

function updateZ(state, number) {
  setZero(state, number === 0);
}

function updateC(state, number) {
  setCarry(state, (number & ~0xFF) !== 0);
}

function updateV(state, number) {
  const bit6 = number >> 6 & 1;
  const bit7 = number >> 7 & 1;
  setOverflow(state, (bit6 ^ bit7) === 1);
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
  const C = getCarry(state) ? 1 : 0;
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
  const M = state.memory[addr.OperandAddress];
  let lsl = M << 1;
  updateC(state, lsl);
  lsl = asByte(lsl);
  updateN(state, lsl);
  updateZ(state, lsl);
  state.registers.A = lsl;
  stepPC(state, addr.nBytes);
}

function opBCC(state, addr) {
  if(!getCarry(state)) {
    setPC(state, addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBCS(state, addr) {
  if(getCarry(state)) {
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
  let xor = A ^ M;
  updateZ(state, xor);
  setNegative(state, (M >> 7 & 1) === 1);
  setOverflow(state, (M >> 6 & 1) === 1);
  stepPC(addr.nBytes);
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

function opBRK(state, addr) {
  throw new Error("TODO");
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
  setCarry(state, false);
  stepPC(state, addr.nBytes);
}

function opCLD(state, addr) {
  setDecimal(state, false);
  stepPC(state, addr.nBytes);
}

function opCLI(state, addr) {
  setIRQDisable(state, false);
  stepPC(state, addr.nBytes);
}

function opCLV(state, addr) {
  setOverflow(state, false);
  stepPC(state, addr.nBytes);
}

function opCMP(state, addr) {
  throw new Error("TODO");
}

function opCPX(state, addr) {
  throw new Error("TODO");
}

function opCPY(state, addr) {
  throw new Error("TODO");
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
  const pcMinusOne = pcAsInteger(state) - 1;
  const PCL = pcMinusOne & 0x00FF;
  const PCH = (pcMinusOne & 0xFF00) >> 8;
  state.memory[state.registers.S] = PCL;
  --state.registers.S;
  state.memory[state.registers.S] = PCH;
  --state.registers.S;
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
  throw new Error("TODO");
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
  throw new Error("TODO");
}

function opPHP(state, addr) {
  throw new Error("TODO");
}

function opPLA(state, addr) {
  throw new Error("TODO");
}

function opPLP(state, addr) {
  throw new Error("TODO");
}

function opROL(state, addr) {
  throw new Error("TODO");
}

function opROR(state, addr) {
  throw new Error("TODO");
}

function opRTI(state, addr) {
  throw new Error("TODO");
}

function opRTS(state, addr) {
  throw new Error("TODO");
}

function opSBC(state, addr) {
  const A = state.registers.A;
  const M = addr.OperandAddress !== undefined
          ? state.memory[addr.OperandAddress]
          : addr.Operand;
  const C = getCarry(state) ? 1 : 0;
  let sum = A + (~M) + C;
  updateC(state, sum);
  sum = asByte(sum);
  updateN(state, sum);
  updateZ(state, sum);
  updateV(state, sum);
  state.registers.A = sum;
  stepPC(state, addr.nBytes);
}

function opSEC(state, addr) {
  setCarry(state, true);
  stepPC(state, addr.nBytes);
}

function opSED(state, addr) {
  setDecimal(state, true);
  stepPC(state, addr.nBytes);
}

function opSEI(state, addr) {
  setIRQDisable(state, true);
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
    /* 0 (LSD) */ [opBRK, asImplied],
    /* 1       */ [opORA, asINDX],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opORA, asZP],
    /* 6       */ [opASL, asZP],
    /* 7       */ null,
    /* 8       */ [opPHP, asImplied],
    /* 9       */ [opORA, asIMM],
    /* A       */ [opASL, asAccum],
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opORA, asABS],
    /* E       */ [opASL, asABS],
    /* F       */ null
  ],
  [ /* 1 */
    /* 0 (LSD) */ [opBPL, asRelative],
    /* 1       */ [opORA, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opORA, asZPX],
    /* 6       */ [opASL, asZPX],
    /* 7       */ null,
    /* 8       */ [opCLC, asImplied],
    /* 9       */ [opORA, asABSY],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opORA, asABSX],
    /* E       */ [opASL, asABSX],
    /* F       */ null
  ],
  [ /* 2 */
    /* 0 (LSD) */ [opJSR, asABS],
    /* 1       */ [opAND, asINDX],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opBIT, asZP],
    /* 5       */ [opAND, asZP],
    /* 6       */ [opROL, asZP],
    /* 7       */ null,
    /* 8       */ [opPLP, asImplied],
    /* 9       */ [opAND, asIMM],
    /* A       */ [opROL, asAccum],
    /* B       */ null,
    /* C       */ [opBIT, asABS],
    /* D       */ [opAND, asABS],
    /* E       */ [opROL, asABS],
    /* F       */ null
  ],
  [ /* 3 */
    /* 0 (LSD) */ [opBMI, asRelative],
    /* 1       */ [opAND, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opAND, asZPX],
    /* 6       */ [opROL, asZPX],
    /* 7       */ null,
    /* 8       */ [opSEC, asImplied],
    /* 9       */ [opAND, asABSY],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opAND, asABSX],
    /* E       */ [opROL, asABSX],
    /* F       */ null
  ],
  [ /* 4 */
    /* 0 (LSD) */ [opRTI, asImplied],
    /* 1       */ [opEOR, asINDX],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opEOR, asZP],
    /* 6       */ [opLSR, asZP],
    /* 7       */ null,
    /* 8       */ [opPHA, asImplied],
    /* 9       */ [opEOR, asIMM],
    /* A       */ [opLSR, asAccum],
    /* B       */ null,
    /* C       */ [opJMP, asABS],
    /* D       */ [opEOR, asABS],
    /* E       */ [opLSR, asABS],
    /* F       */ null
  ],
  [ /* 5 */
    /* 0 (LSD) */ [opBVC, asRelative],
    /* 1       */ [opEOR, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opEOR, asZPX],
    /* 6       */ [opLSR, asZPX],
    /* 7       */ null,
    /* 8       */ [opCLI, asImplied],
    /* 9       */ [opEOR, asABSY],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opEOR, asABSX],
    /* E       */ [opLSR, asABSX],
    /* F       */ null
  ],
  [ /* 6 */
    /* 0 (LSD) */ [opRTS, asImplied],
    /* 1       */ [opADC, asINDX],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opADC, asZP],
    /* 6       */ [opROR, asZP],
    /* 7       */ null,
    /* 8       */ [opPLA, asImplied],
    /* 9       */ [opADC, asIMM],
    /* A       */ [opROR, asAccum],
    /* B       */ null,
    /* C       */ [opJMP, asIndirect],
    /* D       */ [opADC, asABS],
    /* E       */ [opROR, asABS],
    /* F       */ null
  ],
  [ /* 7 */
    /* 0 (LSD) */ [opBVS, asRelative],
    /* 1       */ [opADC, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opADC, asZPX],
    /* 6       */ [opROR, asZPX],
    /* 7       */ null,
    /* 8       */ [opSEI, asImplied],
    /* 9       */ [opADC, asABSY],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opADC, asABSX],
    /* E       */ [opROR, asABSX],
    /* F       */ null
  ],
  [ /* 8 */
    /* 0 (LSD) */ null,
    /* 1       */ [opSTA, asINDX],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opSTY, asZP],
    /* 5       */ [opSTA, asZP],
    /* 6       */ [opSTX, asZP],
    /* 7       */ null,
    /* 8       */ [opDEY, asImplied],
    /* 9       */ null,
    /* A       */ [opTXA, asImplied],
    /* B       */ null,
    /* C       */ [opSTY, asABS],
    /* D       */ [opSTA, asABS],
    /* E       */ [opSTX, asABS],
    /* F       */ null
  ],
  [ /* 9 */
    /* 0 (LSD) */ [opBCC, asRelative],
    /* 1       */ [opSTA, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opSTY, asZPX],
    /* 5       */ [opSTA, asZPX],
    /* 6       */ [opSTX, asZPY],
    /* 7       */ null,
    /* 8       */ [opTYA, asImplied],
    /* 9       */ [opSTA, asABSY],
    /* A       */ [opTXS, asImplied],
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opSTA, asABSX],
    /* E       */ null,
    /* F       */ null
  ],
  [ /* A */
    /* 0 (LSD) */ [opLDY, asIMM],
    /* 1       */ [opLDA, asINDX],
    /* 2       */ [opLDX, asIMM],
    /* 3       */ null,
    /* 4       */ [opLDY, asZP],
    /* 5       */ [opLDA, asZP],
    /* 6       */ [opLDX, asZP],
    /* 7       */ null,
    /* 8       */ [opTAY, asImplied],
    /* 9       */ [opLDA, asIMM],
    /* A       */ [opTAX, asImplied],
    /* B       */ null,
    /* C       */ [opLDY, asABS],
    /* D       */ [opLDA, asABS],
    /* E       */ [opLDX, asABS],
    /* F       */ null
  ],
  [ /* B */
    /* 0 (LSD) */ [opBCS, asRelative],
    /* 1       */ [opLDA, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opLDY, asZPX],
    /* 5       */ [opLDA, asZPX],
    /* 6       */ [opLDX, asZPY],
    /* 7       */ null,
    /* 8       */ [opCLV, asImplied],
    /* 9       */ [opLDA, asABSY],
    /* A       */ [opTSX, asImplied],
    /* B       */ null,
    /* C       */ [opLDY, asABSX],
    /* D       */ [opLDA, asABSX],
    /* E       */ [opLDX, asABSY],
    /* F       */ null
  ],
  [ /* C */
    /* 0 (LSD) */ [opCPY, asIMM],
    /* 1       */ [opCMP, asINDX],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opCPY, asZP],
    /* 5       */ [opCMP, asZP],
    /* 6       */ [opDEC, asZP],
    /* 7       */ null,
    /* 8       */ [opINY, asImplied],
    /* 9       */ [opCMP, asIMM],
    /* A       */ [opDEX, asImplied],
    /* B       */ null,
    /* C       */ [opCPY, asABS],
    /* D       */ [opCMP, asABS],
    /* E       */ [opDEC, asABS],
    /* F       */ null
  ],
  [ /* D */
    /* 0 (LSD) */ [opBNE, asRelative],
    /* 1       */ [opCMP, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opCMP, asZPX],
    /* 6       */ [opDEC, asZPX],
    /* 7       */ null,
    /* 8       */ [opCLD, asImplied],
    /* 9       */ [opCMP, asABSY],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opCMP, asABSX],
    /* E       */ [opDEC, asABSX],
    /* F       */ null
  ],
  [ /* E */
    /* 0 (LSD) */ [opCPX, asIMM],
    /* 1       */ [opSBC, asINDX],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ [opCPX, asZP],
    /* 5       */ [opSBC, asZP],
    /* 6       */ [opINC, asZP],
    /* 7       */ null,
    /* 8       */ [opINX, asImplied],
    /* 9       */ [opSBC, asIMM],
    /* A       */ [opNOP, asImplied],
    /* B       */ null,
    /* C       */ [opCPX, asABS],
    /* D       */ [opSBC, asABS],
    /* E       */ [opINC, asABS],
    /* F       */ null
  ],
  [ /* F */
    /* 0 (LSD) */ [opBEQ, asRelative],
    /* 1       */ [opSBC, asINDY],
    /* 2       */ null,
    /* 3       */ null,
    /* 4       */ null,
    /* 5       */ [opSBC, asZPX],
    /* 6       */ [opINC, asZPX],
    /* 7       */ null,
    /* 8       */ [opSED, asImplied],
    /* 9       */ [opSBC, asABSY],
    /* A       */ null,
    /* B       */ null,
    /* C       */ null,
    /* D       */ [opSBC, asABSX],
    /* E       */ [opINC, asABSX],
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
  process.stdout.write(bToS(getCarry(state)));
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

rl.on('line', (cmd) => {
  if(cmd === 'q') {
    rl.close();
  } else if(cmd === 's') {
    const pc = pcAsInteger(state);
    const instruction = state.memory[pc];
    const msd = (instruction & 0xF0) >> 4;
    const lsd = instruction & 0x0F;
    const [op, addrMode] = opDecodeMatrix[msd][lsd];
    const addr = addrMode(pc, state);
    op(state, addr);
    dumpState(state);
  } else {
    process.stdout.write(`unknown command "${cmd}"`);
  }

  rl.prompt();
}).on('close', () => {
  process.stdout.write("Goodbye.\n");
  process.exit(0);
});
