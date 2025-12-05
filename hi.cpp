// ===============================================================
//  PIECE 1 — Headers, Enums, CPU Structures, Helper Functions
// ===============================================================

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

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
    int cycle[static_cast<int>(Stage::NUM_STAGES)] = {0};
};

// ============================
// Cache statistics structs
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
// Pipeline latch
// ============================
struct PipelineLatch {
    bool valid = false;
    int instrIndex = -1;
    int memCyclesRemaining = 0;  // used for I-cache and D-cache miss stalls
};

// ============================
// CPU state
// ============================
struct CPU {
    uint32_t regs[32] = {0};
    vector<uint32_t> memory;

    vector<Instruction> program;
    unordered_map<string,int> labelToIndex;

    ICache iCache;
    DCache dCache;

    PipelineLatch stage[static_cast<int>(Stage::NUM_STAGES)];
    vector<InstructionTimeline> timelines;

    bool haltFetched = false;

    int cycle = 0;

    // Shared cache port:
    int cacheBusyUntil = 0;

    // Which instruction blocks (pc indices) are cached
    unordered_set<int> icacheBlocks;

    // Used for taken branches — IF must fetch from new PC next cycle
    int programCounterOverride = -1;
};

// ===============================================================
// CACHE HELPERS
// ===============================================================

inline int icacheBlockStart(int pc) {
    return (pc / 4) * 4;
}

bool inICache(const CPU &cpu, int pc) {
    return cpu.icacheBlocks.count(icacheBlockStart(pc)) > 0;
}

void loadICacheBlock(CPU &cpu, int pc) {
    int blk = icacheBlockStart(pc);
    cpu.icacheBlocks.insert(blk);
}

// ===============================================================
// GENERAL HELPERS
// ===============================================================

bool pipelineEmpty(const CPU &cpu) {
    for (int s = 0; s < (int)Stage::NUM_STAGES; ++s) {
        if (cpu.stage[s].valid) return false;
    }
    return true;
}

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
        case Opcode::LW:
            destReg = inst.rd;
            return destReg >= 0;
        default:
            return false;
    }
}

vector<int> getSourceRegs(const Instruction &inst) {
    vector<int> src;
    auto add = [&](int r){ if (r >= 0) src.push_back(r); };

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
            add(inst.rs);
            break;

        case Opcode::SW:
            add(inst.rs);
            add(inst.rd);
            break;

        case Opcode::BEQ:
        case Opcode::BNE:
            add(inst.rs);
            add(inst.rt);
            break;

        default:
            break;
    }
    return src;
}

// When the producer’s value is first forwardable:
int getReadyStage(const Instruction &inst) {
    switch (inst.op) {
        case Opcode::J:
        case Opcode::BEQ:
        case Opcode::BNE:
            return (int)Stage::ID;

        case Opcode::LI:
        case Opcode::AND_:
        case Opcode::OR_:
        case Opcode::SW:
            return (int)Stage::EX1;

        case Opcode::LW:
            return (int)Stage::MEM;

        case Opcode::ADD:
        case Opcode::ADDI:
        case Opcode::SUB:
        case Opcode::SUBI:
            return (int)Stage::EX2;

        case Opcode::MULT:
        case Opcode::MULTI:
            return (int)Stage::EX4;

        case Opcode::DIV_:
        case Opcode::REM_:
            return (int)Stage::EX5;

        default:
            return (int)Stage::MEM;
    }
}

// ===============================================================
//  PIECE 2 — Cache Logic + IF Stage + MEM Stage
// ===============================================================

const int CACHE_MISS_LATENCY = 12;

// Handles D-cache access for LW / SW
void handleMemStage(CPU &cpu, PipelineLatch &cur, PipelineLatch next[], int MEM_IDX, int WB_IDX) {
    int idx = cur.instrIndex;
    Instruction &inst = cpu.program[idx];

    bool isLoad  = (inst.op == Opcode::LW);
    bool isStore = (inst.op == Opcode::SW);

    if (isLoad) {
        // LW CAN MISS → 12-cycle stall
        bool stillWaiting = false;

        // Start miss if not started
        if (cur.memCyclesRemaining <= 0) {
            if (cpu.cycle >= cpu.cacheBusyUntil) {
                cpu.dCache.accessCount++;
                // LW may miss → always treat as miss for simplicity
                cur.memCyclesRemaining = CACHE_MISS_LATENCY;
                cpu.cacheBusyUntil = cpu.cycle + CACHE_MISS_LATENCY;
            } else {
                stillWaiting = true;
            }
        }

        // Continue miss countdown
        if (!stillWaiting && cur.memCyclesRemaining > 0) {
            cur.memCyclesRemaining--;
            if (cur.memCyclesRemaining > 0)
                stillWaiting = true;
        }

        // Stall if not done
        if (stillWaiting) {
            next[MEM_IDX] = cur;
            return;
        }

        // Miss finished → MEM → WB
        if (!next[WB_IDX].valid) {
            cpu.timelines[idx].cycle[MEM_IDX] = cpu.cycle;
            next[WB_IDX] = cur;
        } else {
            next[MEM_IDX] = cur;
        }
    }
    else if (isStore) {
        // SW NEVER MISSES — but still occupies D-cache for ordering, 1 cycle.

        if (cur.memCyclesRemaining <= 0) {
            if (cpu.cycle >= cpu.cacheBusyUntil) {
                cpu.dCache.accessCount++;
                cpu.dCache.hitCount++;     // always hit for SW
                cur.memCyclesRemaining = 1; // store takes 1 cycle
                cpu.cacheBusyUntil = cpu.cycle + 1;
            } else {
                next[MEM_IDX] = cur;
                return;
            }
        }

        cur.memCyclesRemaining--;
        if (cur.memCyclesRemaining > 0) {
            next[MEM_IDX] = cur;
            return;
        }

        if (!next[WB_IDX].valid) {
            cpu.timelines[idx].cycle[MEM_IDX] = cpu.cycle;
            next[WB_IDX] = cur;
        } else {
            next[MEM_IDX] = cur;
        }
    }
    else {
        // Not LW/SW → regular 1-cycle MEM
        if (!next[WB_IDX].valid) {
            cpu.timelines[idx].cycle[MEM_IDX] = cpu.cycle;
            next[WB_IDX] = cur;
        } else {
            next[MEM_IDX] = cur;
        }
    }
}


// ===============================================================
// IF Stage — WITH 12-cycle miss + load 4 instructions
// ===============================================================
void handleIFStage(CPU &cpu, PipelineLatch &cur, PipelineLatch next[],
                   int IF_IDX, int ID_IDX)
{
    if (!cur.valid) return;

    int idx = cur.instrIndex;

    bool hit = inICache(cpu, idx);
    bool stay = false;

    if (!hit) {
        // MISS → must load 4 instructions into cache
        if (cur.memCyclesRemaining <= 0) {
            if (cpu.cycle >= cpu.cacheBusyUntil) {
                cpu.iCache.accessCount++;
                cur.memCyclesRemaining = CACHE_MISS_LATENCY;
                cpu.cacheBusyUntil = cpu.cycle + CACHE_MISS_LATENCY;
            } else {
                stay = true;
            }
        }

        if (!stay && cur.memCyclesRemaining > 0) {
            cur.memCyclesRemaining--;
            if (cur.memCyclesRemaining > 0)
                stay = true;
            else {
                // MISS FINISHED → load 4-inst block
                int base = (idx / 4) * 4;
                cpu.icacheBlocks.insert(base);
                cpu.icacheBlocks.insert(base + 1);
                cpu.icacheBlocks.insert(base + 2);
                cpu.icacheBlocks.insert(base + 3);
                cpu.iCache.hitCount++;  // After miss, next are hits
            }
        }
    }

    // Cannot move to ID if ID is occupied
    if (stay || next[ID_IDX].valid) {
        next[IF_IDX] = cur;
        return;
    }

    // Otherwise IF → ID
    cpu.timelines[idx].cycle[IF_IDX] = cpu.cycle;
    next[ID_IDX] = cur;
}


// ===============================================================
//  PIECE 3 — ID Stage, Hazards, Branch Resolution, EX Pipeline
// ===============================================================

// Returns TRUE if inst writes a register AND sets destReg.
bool writesToReg(const Instruction &inst, int &destReg);

// Returns all registers read by inst.
vector<int> getSourceRegs(const Instruction &inst);

// Returns the pipeline stage at which this instruction’s result is ready.
int getReadyStage(const Instruction &inst);


// ---------------------------------------------------------------
//  Hazard Detection Helper
// ---------------------------------------------------------------
bool hasRAWDependency(const CPU &cpu, int instIndex, const vector<int> &srcs,
                      int ID_IDX)
{
    const int STAGES = static_cast<int>(Stage::NUM_STAGES);

    for (int s = 0; s < STAGES; s++) {
        if (s == ID_IDX) continue;

        const PipelineLatch &other = cpu.stage[s];
        if (!other.valid) continue;

        int olderIdx = other.instrIndex;
        if (olderIdx >= instIndex) continue;  // Only older insts matter

        const Instruction &producer = cpu.program[olderIdx];

        int destReg = -1;
        if (!writesToReg(producer, destReg)) continue;

        int readyStage = getReadyStage(producer);

        // The producer's pipeline stage
        if (s < readyStage) {
            // Value not ready yet → check if inst reads this reg
            for (int r : srcs) {
                if (r == destReg) return true;
            }
        }
    }
    return false;
}


// ---------------------------------------------------------------
//  ID Stage Handler (HLT, branches, hazards, moves to EX1)
// ---------------------------------------------------------------
void handleIDStage(CPU &cpu, PipelineLatch &cur, PipelineLatch next[],
                   int ID_IDX, int EX1_IDX)
{
    if (!cur.valid) return;

    int idx = cur.instrIndex;
    Instruction &inst = cpu.program[idx];

    vector<int> srcs = getSourceRegs(inst);
    bool hazard = hasRAWDependency(cpu, idx, srcs, ID_IDX);

    // ------------------- HLT -------------------
    if (inst.op == Opcode::HLT) {
        if (hazard) {
            next[ID_IDX] = cur;   // stall until safe
            return;
        }

        cpu.timelines[idx].cycle[ID_IDX] = cpu.cycle;
        cpu.haltFetched = true;   // stop future fetches
        return;                   // retire here; do NOT enter EX
    }

    // ------------------- Branches -------------------
    if (inst.op == Opcode::J || inst.op == Opcode::BEQ || inst.op == Opcode::BNE) {

        if (hazard) {
            next[ID_IDX] = cur;  // Need operands available before resolving
            return;
        }

        // Resolve branch now
        cpu.timelines[idx].cycle[ID_IDX] = cpu.cycle;

        bool taken = false;
        if (inst.op == Opcode::J) taken = true;
        else {
            int rs = cpu.regs[inst.rs];
            int rt = cpu.regs[inst.rt];
            if (inst.op == Opcode::BEQ) taken = (rs == rt);
            if (inst.op == Opcode::BNE) taken = (rs != rt);
        }

        if (taken) {
            int newPC = cpu.labelToIndex[inst.label];
            cpu.stage[(int)Stage::IF].valid = false;   // flush IF
            cpu.stage[(int)Stage::IF].instrIndex = -1;
            // PC will be updated in simulate() when fetch runs
            cpu.programCounterOverride = newPC;
        }

        return;  // branches never go into EX
    }

    // ------------------- Normal instructions → EX1 -------------------
    bool ex1Free = (!next[EX1_IDX].valid);

    if (!hazard && ex1Free) {
        cpu.timelines[idx].cycle[ID_IDX] = cpu.cycle;
        next[EX1_IDX] = cur;
    }
    else {
        next[ID_IDX] = cur;  // Stall
    }
}


// ---------------------------------------------------------------
//  EX Pipeline Handler (EX1 → EX5)
// ---------------------------------------------------------------
void handleEXStages(CPU &cpu, PipelineLatch next[],
                    int EX1_IDX, int EX5_IDX)
{
    for (int s = EX5_IDX; s >= EX1_IDX; s--) {
        PipelineLatch &cur = cpu.stage[s];
        if (!cur.valid) continue;

        int idx = cur.instrIndex;

        int nextStage = s + 1;
        if (nextStage < (int)Stage::NUM_STAGES &&
            !next[nextStage].valid)
        {
            cpu.timelines[idx].cycle[s] = cpu.cycle;
            next[nextStage] = cur;
        }
        else {
            next[s] = cur; // stall in stage
        }
    }
}


// ===============================================================
//  PIECE 4 — Full simulate() and Output Writer
// ===============================================================

void simulate(CPU &cpu, const string &outputFile)
{
    const int CACHE_LATENCY = 12;     // 12-cycle miss stall
    const int STAGES        = static_cast<int>(Stage::NUM_STAGES);
    const int IF_IDX        = static_cast<int>(Stage::IF);
    const int ID_IDX        = static_cast<int>(Stage::ID);
    const int EX1_IDX       = static_cast<int>(Stage::EX1);
    const int EX5_IDX       = static_cast<int>(Stage::EX5);
    const int MEM_IDX       = static_cast<int>(Stage::MEM);
    const int WB_IDX        = static_cast<int>(Stage::WB);

    cpu.cycle = 0;
    cpu.haltFetched = false;
    cpu.cacheBusyUntil = 0;

    int pc = 0;
    int N = (int)cpu.program.size();
    cpu.programCounterOverride = -1;

    while (true) {
        cpu.cycle++;

        PipelineLatch next[STAGES];
        for (int s = 0; s < STAGES; s++) next[s] = PipelineLatch{};

        // ---------------- WB ----------------
        {
            auto &cur = cpu.stage[WB_IDX];
            if (cur.valid) {
                int idx = cur.instrIndex;
                cpu.timelines[idx].cycle[WB_IDX] = cpu.cycle;
            }
        }

        // ---------------- MEM (D-cache) ----------------
        {
            auto &cur = cpu.stage[MEM_IDX];
            if (cur.valid) {

                int idx = cur.instrIndex;
                Instruction &inst = cpu.program[idx];

                bool isLoadStore =
                    (inst.op == Opcode::LW || inst.op == Opcode::SW);

                if (isLoadStore) {
                    bool stay = false;

                    if (inst.op == Opcode::LW) {
                        // LW *can miss* — must start D-cache access
                        if (cur.memCyclesRemaining <= 0) {
                            if (cpu.cycle >= cpu.cacheBusyUntil) {
                                cpu.dCache.accessCount++;
                                cpu.dCache.hitCount++; // simulator assumes hits

                                cur.memCyclesRemaining = CACHE_LATENCY;
                                cpu.cacheBusyUntil = cpu.cycle + CACHE_LATENCY;
                            } else {
                                stay = true;
                            }
                        }
                    }

                    // SW NEVER misses → 0-cycle D-cache penalty
                    if (inst.op == Opcode::SW) {
                        cur.memCyclesRemaining = 0;
                    }

                    if (!stay && cur.memCyclesRemaining > 0) {
                        cur.memCyclesRemaining--;
                        if (cur.memCyclesRemaining > 0) stay = true;
                    }

                    if (stay) {
                        next[MEM_IDX] = cur;
                    }
                    else {
                        if (!next[WB_IDX].valid) {
                            cpu.timelines[idx].cycle[MEM_IDX] = cpu.cycle;
                            next[WB_IDX] = cur;
                        } else {
                            next[MEM_IDX] = cur;
                        }
                    }
                }
                else {
                    // Non-load/store = 1-cycle MEM
                    if (!next[WB_IDX].valid) {
                        cpu.timelines[idx].cycle[MEM_IDX] = cpu.cycle;
                        next[WB_IDX] = cur;
                    } else {
                        next[MEM_IDX] = cur;
                    }
                }
            }
        }

        // ---------------- EX1..EX5 ----------------
        handleEXStages(cpu, next, EX1_IDX, EX5_IDX);

        // ---------------- ID ----------------
        handleIDStage(cpu, cpu.stage[ID_IDX], next, ID_IDX, EX1_IDX);

        // ---------------- IF (I-cache access) ----------------
        {
            auto &cur = cpu.stage[IF_IDX];
            if (cur.valid) {
                int idx = cur.instrIndex;
                bool stay = false;

                bool hit = inICache(cpu, idx);

                // MISS → must start 12-cycle memory fill
                if (!hit) {
                    if (cur.memCyclesRemaining <= 0) {
                        if (cpu.cycle >= cpu.cacheBusyUntil) {
                            //cpu.iCache.accessCount++;

                            cur.memCyclesRemaining = CACHE_LATENCY;
                            cpu.cacheBusyUntil = cpu.cycle + CACHE_LATENCY;
                        }
                        else {
                            stay = true;
                        }
                    }

                    if (!stay && cur.memCyclesRemaining > 0) {
                        cur.memCyclesRemaining--;
                        if (cur.memCyclesRemaining > 0) stay = true;
                        else {
                            // fill ends → load block
                            loadICacheBlock(cpu, idx);
                        }
                    }
                }
                else {
                    cpu.iCache.hitCount++;
                }

                if (stay || next[ID_IDX].valid) {
                    next[IF_IDX] = cur;
                }
                else {
                    cpu.timelines[idx].cycle[IF_IDX] = cpu.cycle;
                    cur.memCyclesRemaining = 0;
                    next[ID_IDX] = cur;
                    cpu.iCache.accessCount++;
                }
            }
        }

        // ---------------- FETCH NEW INSTRUCTION ----------------
        if (!cpu.haltFetched) {
            if (!next[IF_IDX].valid) {

                if (cpu.programCounterOverride >= 0) {
                    pc = cpu.programCounterOverride;
                    cpu.programCounterOverride = -1;
                }

                if (pc < N) {
                    PipelineLatch L;
                    L.valid = true;
                    L.instrIndex = pc;
                    next[IF_IDX] = L;
                    pc++;
                }
            }
        }

        // ---------------- COMMIT NEXT ----------------
        for (int s = 0; s < STAGES; s++) cpu.stage[s] = next[s];

        // ---------------- STOP WHEN DONE ----------------
        if (pc >= N && pipelineEmpty(cpu)) break;
    }

    // =====================================================
    // Write Output File (EXACT FORMAT YOU REQUESTED)
    // =====================================================
    ofstream fout(outputFile);
    if (!fout) {
        cerr << "ERR: Cannot open output\n";
        return;
    }

    fout << "Cycle Number for Each Stage"
         << setw(12) << "IF"
         << setw(8)  << "ID"
         << setw(8)  << "EX5"
         << setw(8)  << "MEM"
         << setw(8)  << "WB"
         << "\n";

    for (int i = 0; i < N; i++) {
        const Instruction &inst = cpu.program[i];
        const InstructionTimeline &tl = cpu.timelines[i];

        fout << left << setw(30) << inst.raw
             << right << setw(8) << ((tl.cycle[IF_IDX] - 1) != -1 ? std::to_string(tl.cycle[IF_IDX] - 1) : "")
             << right << setw(8) << ((tl.cycle[ID_IDX] - 1) != -1 ? std::to_string(tl.cycle[ID_IDX] - 1) : "")
             << right << setw(8) << ((tl.cycle[EX5_IDX] - 1) != -1 ? std::to_string(tl.cycle[EX5_IDX] - 1) : "")
             << right << setw(8) << ((tl.cycle[MEM_IDX] - 1) != -1 ? std::to_string(tl.cycle[MEM_IDX] - 1) : "")
             << right << setw(8) << ((tl.cycle[WB_IDX] - 1) != -1 ? std::to_string(tl.cycle[WB_IDX] - 1) : "")
             << "\n";
    }

    fout << "Total number of access requests for instruction cache: "
         << cpu.iCache.accessCount << "\n";
    fout << "Number of instruction cache hits: " << cpu.iCache.hitCount << "\n";
    fout << "Total number of access requests for data cache: "
         << cpu.dCache.accessCount << "\n";
    fout << "Number of data cache hits: " << cpu.dCache.hitCount << "\n";
}


// ===============================================================
//  FINAL main() — unchanged format
// ===============================================================
int main(int argc, char *argv[])
{
    if (argc != 4) {
        cerr << "Usage: simulator inst.txt data.txt output.txt\n";
        return 1;
    }

    string instFile = argv[1];
    string dataFile = argv[2];
    string outputFile = argv[3];

    CPU cpu;

    // load program
    cpu.program = parseInstFile(instFile, cpu.labelToIndex);

    // load data
    auto dataVec = parseDataFile(dataFile);
    cpu.memory.assign(2048, 0);

    const uint32_t BASE = 0x100;
    for (size_t i = 0; i < dataVec.size(); i++) {
        cpu.memory[BASE + i] = dataVec[i];
    }

    cpu.timelines.resize(cpu.program.size());

    simulate(cpu, outputFile);

    return 0;
}
