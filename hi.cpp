#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>

#include "parseInput.hpp"   // must provide: Opcode, Instruction, parseInstFile, parseDataFile

using namespace std;

// ============================
// Pipeline stages
// ============================
enum class Stage {
    IF = 0,
    ID,
    EX1,
    EX2,
    EX3,
    EX4,
    EX5,
    MEM,
    WB,
    NUM_STAGES
};

// ============================
// Timeline for each instruction
// ============================
struct InstructionTimeline {
    // cycle when the instruction LEAVES each stage
    int cycle[static_cast<int>(Stage::NUM_STAGES)] = {0};
};

// ============================
// Very simple cache stubs
// ============================
struct ICache {
    int accessCount = 0;
    int hitCount    = 0;
};

struct DCache {
    int accessCount = 0;
    int hitCount    = 0;
};

// ============================
// Pipeline latch between stages
// ============================
struct PipelineLatch {
    bool valid = false;
    int  instrIndex = -1;       // index into program vector

    // For MEM stage: dummy stall counter for LW/SW
    int  memCyclesRemaining = 0;
};

// ============================
// CPU state
// ============================
struct CPU {
    uint32_t regs[32] = {0};

    // word-addressed memory
    vector<uint32_t> memory;

    // program and label map (filled by parser)
    vector<Instruction> program;
    unordered_map<string,int> labelToIndex;

    // caches (stubs for now)
    ICache iCache;
    DCache dCache;

    // pipeline latches
    PipelineLatch stage[static_cast<int>(Stage::NUM_STAGES)];

    // per-instruction timelines
    vector<InstructionTimeline> timelines;

    bool haltFetched = false;   // true after HLT passes ID
    bool done        = false;

    int cycle = 0;
    // NEW: shared cache resource (I-cache + D-cache)
    int cacheBusyUntil = 0; // cycle when the cache port becomes free
    unordered_set<int> icacheBlocks;
};

bool inICache(const CPU &cpu, int pc) {
    int blockStart = (pc / 4) * 4;
    return cpu.icacheBlocks.count(blockStart) > 0;
}

void loadICacheBlock(CPU &cpu, int pc) {
    int blockStart = (pc / 4) * 4;
    cpu.icacheBlocks.insert(blockStart);
}

// ============================
// Helper: check if all pipeline stages are empty
// ============================
bool pipelineEmpty(const CPU &cpu) {
    for (int s = 0; s < static_cast<int>(Stage::NUM_STAGES); ++s) {
        if (cpu.stage[s].valid) return false;
    }
    return true;
}

// Return TRUE if this instruction writes to a register, and set destReg to that register number.
bool writesToReg(const Instruction &inst, int &destReg) {
    destReg = -1;
    switch (inst.op) {
        case Opcode::LI:
        case Opcode::ADD:
        case Opcode::ADDI:
        case Opcode::SUB:
        case Opcode::SUBI:
        case Opcode::AND_:
        case Opcode::OR_:
        case Opcode::MULT:
        case Opcode::MULTI:
        case Opcode::DIV_:
        case Opcode::REM_:
        case Opcode::LW:   // load writes to rd
            destReg = inst.rd;
            return (destReg >= 0);
        default:
            // SW, BEQ, BNE, J, HLT do not write a destination register
            return false;
    }
}

// Collect all source registers that this instruction READS.
vector<int> getSourceRegs(const Instruction &inst) {
    vector<int> srcs;
    auto add = [&](int r) {
        if (r >= 0) srcs.push_back(r);
    };

    switch (inst.op) {
        case Opcode::ADD:
        case Opcode::SUB:
        case Opcode::AND_:
        case Opcode::OR_:
        case Opcode::MULT:
        case Opcode::DIV_:
        case Opcode::REM_:
            add(inst.rs);
            add(inst.rt);
            break;

        case Opcode::ADDI:
        case Opcode::SUBI:
        case Opcode::MULTI:
            add(inst.rs);
            break;

        case Opcode::LW:
            // LW Rdest, offset(Rbase) -> reads base register
            add(inst.rs);
            break;

        case Opcode::SW:
            // SW Rsrc, offset(Rbase) -> reads base and source
            add(inst.rs); // base
            add(inst.rd); // value to store (we used rd in parser)
            break;

        case Opcode::BEQ:
        case Opcode::BNE:
            add(inst.rs);
            add(inst.rt);
            break;

        // LI, J, HLT: no true data dependencies in this model
        default:
            break;
    }

    return srcs;
}

// Stage index (Stage::IF, ID, EX1, ...) at which this instruction's
// destination register value is first available for forwarding.
int getReadyStage(const Instruction &inst) {
    switch (inst.op) {
        // These don't write regs, but if they did, they'd be ready in ID.
        case Opcode::J:
        case Opcode::BEQ:
        case Opcode::BNE:
            return static_cast<int>(Stage::ID);

        // LI: ready after EX1
        case Opcode::LI:
            return static_cast<int>(Stage::EX1);

        // AND / OR / SW: ALU result is ready after EX1
        case Opcode::AND_:
        case Opcode::OR_:
        case Opcode::SW:
            return static_cast<int>(Stage::EX1);

        // LW: loaded data is available only in MEM (then forwarded)
        case Opcode::LW:
            return static_cast<int>(Stage::MEM);

        // ADD-family: 2-cycle EX ⇒ result ready after EX2
        case Opcode::ADD:
        case Opcode::ADDI:
        case Opcode::SUB:
        case Opcode::SUBI:
            return static_cast<int>(Stage::EX2);

        // MULT-family: 4-cycle EX ⇒ result ready after EX4
        case Opcode::MULT:
        case Opcode::MULTI:
            return static_cast<int>(Stage::EX4);

        // DIV / REM: 5-cycle EX ⇒ result ready after EX5
        case Opcode::DIV_:
        case Opcode::REM_:
            return static_cast<int>(Stage::EX5);

        default:
            // Safe fallback
            return static_cast<int>(Stage::MEM);
    }
}

// ============================
// Pipeline simulation
//  - instructions flow through pipeline
//  - MEM stubs: LW/SW just "stall" for fixed latency
// ============================
void simulate(CPU &cpu, const string &outputFile) {
    //const int DUMMY_MEM_LATENCY = 1;  // # of cycles LW/SW spend stalling in MEM
    const int CACHE_LATENCY = 9;

    cpu.cycle = 0;
    cpu.done  = false;

    const int STAGES  = static_cast<int>(Stage::NUM_STAGES);
    const int IF_IDX  = static_cast<int>(Stage::IF);
    const int ID_IDX  = static_cast<int>(Stage::ID);
    const int EX1_IDX = static_cast<int>(Stage::EX1);
    const int EX5_IDX = static_cast<int>(Stage::EX5);
    const int MEM_IDX = static_cast<int>(Stage::MEM);
    const int WB_IDX  = static_cast<int>(Stage::WB);

    // PC is index into cpu.program
    int pc = 0;
    const int N = static_cast<int>(cpu.program.size());

    // main simulation loop
    while (true) {
        cpu.cycle++;

        // next-cycle latches
        PipelineLatch next[STAGES];
        for (int s = 0; s < STAGES; ++s) {
            next[s] = PipelineLatch{};
        }

        // -------------------------
        // 1) WB stage
        // -------------------------
        {
            PipelineLatch &cur = cpu.stage[WB_IDX];
            if (cur.valid) {
                int idx = cur.instrIndex;
                // mark time leaving WB
                cpu.timelines[idx].cycle[WB_IDX] = cpu.cycle;
                // retired
            }
        }

        // -------------------------
// 2) MEM stage (D-cache for LW/SW, 9 cycles, shared with I-cache)
// -------------------------
{
    PipelineLatch &cur = cpu.stage[MEM_IDX];
    if (cur.valid) {
        int idx = cur.instrIndex;
        Instruction &inst = cpu.program[idx];

        bool isLoadStore = (inst.op == Opcode::LW || inst.op == Opcode::SW);

        if (isLoadStore) {
            bool stayInMEM = false;

            // If we haven't started a D-cache access yet
            if (cur.memCyclesRemaining <= 0) {
                // Try to start a D-cache access (shared port)
                if (cpu.cycle >= cpu.cacheBusyUntil) {
                    cpu.dCache.accessCount++;
                    cpu.dCache.hitCount++;  // assume all hits

                    cur.memCyclesRemaining = CACHE_LATENCY;
                    cpu.cacheBusyUntil     = cpu.cycle + CACHE_LATENCY;
                } else {
                    // Cache port busy (I-cache or other D-cache access)
                    stayInMEM = true;
                }
            }

            if (!stayInMEM && cur.memCyclesRemaining > 0) {
                // Access in progress
                cur.memCyclesRemaining--;

                if (cur.memCyclesRemaining > 0) {
                    // Still waiting on D-cache
                    stayInMEM = true;
                }
            }

            if (stayInMEM) {
                next[MEM_IDX] = cur;
            } else {
                // D-cache access done, try to move MEM -> WB
                if (!next[WB_IDX].valid) {
                    cpu.timelines[idx].cycle[MEM_IDX] = cpu.cycle;
                    cur.memCyclesRemaining = 0;
                    next[WB_IDX] = cur;
                } else {
                    // WB busy, remain in MEM
                    next[MEM_IDX] = cur;
                }
            }
        } else {
            // Non-memory instruction: simple 1-cycle MEM
            if (!next[WB_IDX].valid) {
                cpu.timelines[idx].cycle[MEM_IDX] = cpu.cycle;
                next[WB_IDX] = cur;
            } else {
                next[MEM_IDX] = cur;
            }
        }
    }
}

        // -------------------------
        // 3) EX5..EX1 (backwards, everyone flows EX1→EX5)
// -------------------------
        {
            for (int s = EX5_IDX; s >= EX1_IDX; --s) {
                PipelineLatch &cur = cpu.stage[s];
                if (!cur.valid) continue;

                int idx       = cur.instrIndex;
                int nextStage = s + 1;

                if (nextStage < STAGES && !next[nextStage].valid) {
                    cpu.timelines[idx].cycle[s] = cpu.cycle;
                    next[nextStage] = cur;
                } else {
                    // blocked: stay here
                    next[s] = cur;
                }
            }
        }

        // -------------------------
        // 4) ID stage (HLT + branches retire here, others go to EX1)
// -------------------------
        {
            PipelineLatch &cur = cpu.stage[ID_IDX];
            if (cur.valid) {
                int idx = cur.instrIndex;
                Instruction &inst = cpu.program[idx];

                // 1) Collect source registers (works for normal ops AND BEQ/BNE)
                vector<int> srcs = getSourceRegs(inst);
                bool hasHazard   = false;

                // 2) Scan all pipeline stages for older writers that conflict
                for (int s = 0; s < STAGES && !hasHazard; ++s) {
                    if (s == ID_IDX) continue;  // skip self

                    PipelineLatch &otherLatch = cpu.stage[s];
                    if (!otherLatch.valid) continue;

                    int otherIdx = otherLatch.instrIndex;
                    if (otherIdx >= idx) continue;  // only older instructions can cause RAW hazards

                    Instruction &producer = cpu.program[otherIdx];

                    int destReg = -1;
                    if (!writesToReg(producer, destReg)) continue; // no write → no RAW

                    // At which stage is this producer's value first available?
                    int readyStage    = getReadyStage(producer);
                    int producerStage = s;

                    bool producerReady = (producerStage >= readyStage);

                    if (!producerReady) {
                        // Check if this destReg is one of our sources
                        for (int src : srcs) {
                            if (src == destReg) {
                                hasHazard = true;
                                break;
                            }
                        }
                    }
                }

                // -------- Special case: HLT --------
                if (inst.op == Opcode::HLT) {
                    // HLT doesn’t read regs in this model, so hasHazard should be false,
                    // but we can still be safe and stall if somehow true.
                    if (hasHazard) {
                        next[ID_IDX] = cur;  // stay in ID until deps are clear
                    } else {
                        cpu.timelines[idx].cycle[ID_IDX] = cpu.cycle;  // retires in ID
                        cpu.haltFetched = true;                        // stop fetching
                        // Do NOT enqueue it into any next[] stage; it vanishes here.
                    }
                }
                // -------- Special case: Branches (J, BEQ, BNE) --------
                else if (inst.op == Opcode::J ||
                         inst.op == Opcode::BEQ ||
                         inst.op == Opcode::BNE) {
                    // Branch should only resolve when its source registers are ready.
                    if (hasHazard) {
                        // Not all source regs are ready yet -> branch must wait in ID
                        next[ID_IDX] = cur;
                    } else {
                        // Operands are ready -> resolve branch here in ID.
                        cpu.timelines[idx].cycle[ID_IDX] = cpu.cycle;

                        // TODO (later):
                        //  - evaluate BEQ/BNE condition using cpu.regs[rs], cpu.regs[rt]
                        //  - set pc = target if taken
                        //  - flush IF latch on taken branch

                        // For now we just retire the branch here and DO NOT send it to EX/MEM/WB.
                        // So we leave next[ID_IDX] invalid.
                    }
                }
                // -------- Normal instructions: go into EX1, using same hazard logic --------
                else {
                    int  nextStage  = EX1_IDX; // EX1
                    bool targetFree = !next[nextStage].valid;
                    bool dataReady  = !hasHazard;

                    if (dataReady && targetFree) {
                        // No hazard and EX1 free: move ID -> EX1
                        cpu.timelines[idx].cycle[ID_IDX] = cpu.cycle;
                        next[nextStage] = cur;
                    } else {
                        // Stall in ID
                        next[ID_IDX] = cur;
                    }
                }
            }
        }

        // -------------------------
// 5) IF stage (I-cache access, 9 cycles, shared with D-cache)
// -------------------------
{
    PipelineLatch &cur = cpu.stage[IF_IDX];
    if (cur.valid) {
        int idx       = cur.instrIndex;
        int nextStage = ID_IDX;

        bool stayInIF = false;
        bool isHit    = inICache(cpu, idx);

        // If not in cache → MISS → must start or continue a cache fill
        if (!isHit) {
            // If not already filling (memCyclesRemaining == 0)
            if (cur.memCyclesRemaining <= 0) {
                // Try to start cache fill
                if (cpu.cycle >= cpu.cacheBusyUntil) {

                    // Start fill
                    cpu.iCache.accessCount++;
                    cur.memCyclesRemaining = 9;  // CACHE_LATENCY
                    cpu.cacheBusyUntil     = cpu.cycle + 9;

                } else {
                    // Cache port busy, must wait
                    stayInIF = true;
                }
            }

            // If fill is in progress
            if (!stayInIF && cur.memCyclesRemaining > 0) {
                cur.memCyclesRemaining--;
                if (cur.memCyclesRemaining > 0) {
                    stayInIF = true;   // still filling
                } else {
                    // Fill finished this cycle
                    loadICacheBlock(cpu, idx);   // <-- Insert block of 4 instructions
                }
            }
        }

        // If miss OR ID is busy → stay in IF
        if (stayInIF || next[nextStage].valid) {
            next[IF_IDX] = cur;
        }
        else {
            // Cache hit AND ID free → move forward
            cpu.timelines[idx].cycle[IF_IDX] = cpu.cycle;
            cur.memCyclesRemaining = 0;
            next[nextStage] = cur;
        }
    }
}
        // -------------------------
        // 6) Fetch new instruction into IF
        // -------------------------
        if (!cpu.haltFetched) {
            if (!next[IF_IDX].valid && pc < N) {
                PipelineLatch latch;
                latch.valid             = true;
                latch.instrIndex        = pc;
                latch.memCyclesRemaining = 0;

                next[IF_IDX] = latch;
                pc++;
            }
        }

        // commit next latches
        for (int s = 0; s < STAGES; ++s) {
            cpu.stage[s] = next[s];
        }

        // stop when no more instructions to fetch AND pipeline drained
        if (pc >= N && pipelineEmpty(cpu)) {
            break;
        }
    }

    // ============================
    // Write output file
    // ============================
    ofstream fout(outputFile);
    if (!fout) {
        cerr << "Error: cannot open output file: " << outputFile << "\n";
        exit(1);
    }

    fout << "Cycle Number for Each Stage"
         << setw(12) << "IF"
         << setw(8)  << "ID"
         << setw(8)  << "EX5"
         << setw(8)  << "MEM"
         << setw(8)  << "WB"
         << "\n";

    for (size_t i = 0; i < cpu.program.size(); ++i) {
        const Instruction         &inst = cpu.program[i];
        const InstructionTimeline &tl   = cpu.timelines[i];

        fout << left  << setw(30) << inst.raw   // first column: instruction text
             << right << setw(8)  << tl.cycle[IF_IDX]
             << right << setw(8)  << tl.cycle[ID_IDX]
             << right << setw(8)  << tl.cycle[EX5_IDX]
             << right << setw(8)  << tl.cycle[MEM_IDX]
             << right << setw(8)  << tl.cycle[WB_IDX]
             << "\n";
    }

    fout << "Total number of access requests for instruction cache: "
         << cpu.iCache.accessCount << "\n";
    fout << "Number of instruction cache hits: " << cpu.iCache.hitCount << "\n";
    fout << "Total number of access requests for data cache: "
         << cpu.dCache.accessCount << "\n";
    fout << "Number of data cache hits: " << cpu.dCache.hitCount << "\n";
}

// ============================
// main()
// ============================
int main(int argc, char *argv[]) {
    if (argc != 4) {
        cerr << "Usage: simulator inst.txt data.txt output.txt\n";
        return 1;
    }

    string instFile   = argv[1];
    string dataFile   = argv[2];
    string outputFile = argv[3];

    CPU cpu;

    // Parse instructions (fills program + label map)
    cpu.program = parseInstFile(instFile, cpu.labelToIndex);

    // Parse data file into a temp vector
    auto dataVec = parseDataFile(dataFile);

    // Make memory big enough and load data at address 0x100
    cpu.memory.assign(1024, 0);
    const uint32_t DATA_BASE_ADDR = 0x100;

    for (size_t i = 0; i < dataVec.size(); ++i) {
        uint32_t addr = DATA_BASE_ADDR + static_cast<uint32_t>(i);
        if (addr >= cpu.memory.size()) {
            cerr << "Error: data segment overflow in memory\n";
            return 1;
        }
        cpu.memory[addr] = dataVec[i];
    }

    // Prepare timelines (one per instruction)
    cpu.timelines.resize(cpu.program.size());

    // Run pipeline with dummy MEM stalls
    simulate(cpu, outputFile);

    return 0;
}
