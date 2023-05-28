function initialMachineState() {
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
  const memory = new Uint8Array(65535);

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
  let operandAddress = 0; /* 16b */
  operandAddress += zp;
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
  let operandAddress = 0; /* 16b */
  operandAddress += zp + state.registers.X;
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
  let operandAddress = 0; /* 16b */
  operandAddress += zp + state.registers.Y;
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
  const effectiveAddress = operandAddress[0] + state.registers.X;
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
  const effectiveAddress = operandAddress[0] + state.registers.Y;
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
  const offset = state.memory[address + 1];
  let pc = 0; /* 16b */
  pc  = state.registers.PCL;
  pc += state.registers.PCH << 8;
  const effectiveAddress = pc + offset;
  return {
    nBytes: 2,
    NewPCValue: effectiveAddress
  }
}

function asStack(address, state) {
  /* The Stack may use memory from 0100 to 01FF and the
   * effective address of the Stack address mode will always
   * be within this range. Stack addressing refers to all
   * instructions that push or pull data from the stack, such
   * as Push, Pull, Jump to Subroutine, Return from Subroutine,
   * Interrupts and Return from
   * Interrupt. */
  let operandAddress = 0; /* 16b */
  operandAddress += 0x0100 + state.registers.S;
  return {
    nBytes: 1,
    OperandAddress: operandAddress
  }
}

function asINDX(address, state) {
  /* The Zero Page Indexed Indirect addressing mode is often
   * referred to as Indirect,X. The second byte of the
   * instruction is the zero page address to which the X Index
   * Register is added and the result points to the low byte of
   * the indirect address. */
  const zp = state.memory[address + 1];
  /* FIXME: what if zp + state.registers.X is more than 0xFF? */
  const indirectAddress = zp + state.registers.X;
  let operandAddress = 0; /* 16b */
  operandAddress += state.memory[indirectAddress];
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
  const operandAddress = indirectBaseAddress + state.registers.Y;
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

function clamp8bits(number) {
  return number & 0xFF;
}

function stepPC(state, bytes) {
  let pc = (state.registers.PCH << 8) + state.registers.PCL;
  pc = (pc + bytes) % 0xFFFF;
  setPC(pc);
}

function setPC(state, newPC) {
  state.registers.PCL = newPC & 0xFF;
  state.registers.PCH = (newPC & 0xFF00) >> 8;
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
  sum = clamp8bits(sum);
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
  lsl = clamp8bits(lsl);
  updateN(state, lsl);
  updateZ(state, lsl);
  state.registers.A = lsl;

  stepPC(state, addr.nBytes);
}

function opBCC(state, addr) {
  if(!getCarry(state)) {
    setPC(addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBCS(state, addr) {
  if(getCarry(state)) {
    setPC(addr.NewPCValue);
  } else {
    stepPC(state, addr.nBytes);
  }
}

function opBEQ(state, addr) {
  if(getZero(state)) {
    setPC(addr.NewPCValue);
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
  throw new Error("TODO");
}

function opCLD(state, addr) {
  throw new Error("TODO");
}

function opCLI(state, addr) {
  throw new Error("TODO");
}

function opCLV(state, addr) {
  throw new Error("TODO");
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
  throw new Error("TODO");
}

function opDEX(state, addr) {
  throw new Error("TODO");
}

function opDEY(state, addr) {
  throw new Error("TODO");
}

function opEOR(state, addr) {
  throw new Error("TODO");
}

function opINC(state, addr) {
  throw new Error("TODO");
}

function opINX(state, addr) {
  throw new Error("TODO");
}

function opINY(state, addr) {
  throw new Error("TODO");
}

function opJMP(state, addr) {
  throw new Error("TODO");
}

function opJSR(state, addr) {
  throw new Error("TODO");
}

function opLDA(state, addr) {
  throw new Error("TODO");
}

function opLDX(state, addr) {
  throw new Error("TODO");
}

function opLDY(state, addr) {
  throw new Error("TODO");
}

function opLSR(state, addr) {
  throw new Error("TODO");
}

function opNOP(state, addr) {
  throw new Error("TODO");
}

function opORA(state, addr) {
  throw new Error("TODO");
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
  sum = clamp8bits(sum);
  updateN(state, sum);
  updateZ(state, sum);
  updateV(state, sum);
  state.registers.A = sum;
}

function opSEC(state, addr) {
  throw new Error("TODO");
}

function opSED(state, addr) {
  throw new Error("TODO");
}

function opSEI(state, addr) {
  throw new Error("TODO");
}

function opSTA(state, addr) {
  throw new Error("TODO");
}

function opSTX(state, addr) {
  throw new Error("TODO");
}

function opSTY(state, addr) {
  throw new Error("TODO");
}

function opTAX(state, addr) {
  throw new Error("TODO");
}

function opTAY(state, addr) {
  throw new Error("TODO");
}

function opTSX(state, addr) {
  throw new Error("TODO");
}

function opTXA(state, addr) {
  throw new Error("TODO");
}

function opTXS(state, addr) {
  throw new Error("TODO");
}

function opTYA(state, addr) {
  throw new Error("TODO");
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
