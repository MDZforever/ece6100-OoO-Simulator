#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <inttypes.h>

#include "procsim.hpp"
#include <deque>

// -------------------------------------------------------------------
// Internal data structure definitions

// Number of architectural registers is defined in procsim.hpp (assumed to be 32)
#ifndef NUM_REGS
#define NUM_REGS 32
#endif

// Physical Register File Entry
typedef struct {
    bool free;   // Whether this entry can be assgined to a dest_reg.
    bool ready;  // Whether the current data is done caculating by FU. 
} PRFEntry;

// Reservation Station entry (for scheduling queue)
typedef struct {
    bool valid;
    bool fired;
    inst_t *inst;
    int fu_type; // 0: ALU (including branch), 1: MUL, 2: LSU
    int dest_preg; // allocated physical register for destination; -1 if none
    int src1_preg;
    int src2_preg;
    bool src1_ready;
    bool src2_ready;
    int remaining_cycles; // execution latency remaining
} ReservationStation;

// ROB entry definition
typedef struct {
    bool valid;
    bool completed;
    bool mispredict;
    uint64_t dyn_count;
    int dest_preg; // allocated destination physical register (-1 if none)
    int prev_preg; // previous mapping for destination (if any)
    inst_t *inst;
} ROBEntry;

// -------------------------------------------------------------------
// Global pipeline structures and state variables (static for this file)

static procsim_conf_t current_conf;
static procsim_stats_t *current_stats;

// Infinite dispatch queue; popping the head must not shift the entire backlog.
static std::deque<inst_t *> dispatch_queue;

// Scheduling (reservation stations) queue
static ReservationStation *sched_queue = NULL;
static size_t sched_queue_capacity = 0;
static size_t sched_queue_count = 0;

// Reorder Buffer (ROB) implemented as a circular queue
static ROBEntry *ROB = NULL;
static size_t ROB_capacity = 0;
static size_t ROB_count = 0;
static size_t ROB_head = 0;
static size_t ROB_tail = 0;

// Register Alias Table (RAT) for NUM_REGS registers
static int RAT[NUM_REGS];

// Physical Register File (PRF)
static PRFEntry *PRF = NULL;
static size_t PRF_size = 0; // = NUM_REGS + current_conf.num_pregs

#ifdef DEBUG
static void print_rat() {
    for (int i = 0; i < NUM_REGS; ++i)
        printf("RAT[%d] = %d\n", i, RAT[i]);
}

static void print_prf() {
    for (size_t i = 0; i < PRF_size; ++i)
        printf("PRF[%zu]: free=%d ready=%d\n", i, PRF[i].free, PRF[i].ready);
}

static void print_rob() {
    for (size_t i = 0; i < ROB_count; ++i) {
        const ROBEntry &entry = ROB[(ROB_head + i) % ROB_capacity];
        printf("ROB[%zu]: instruction=%" PRIu64 " completed=%d\n",
               i, entry.dyn_count, entry.completed);
    }
}
#endif

// -------------------------------------------------------------------
// Allocate the lowest available physical register, as required by the model.
static bool allocate_free_reg(int *preg_out) {
    for (size_t i = NUM_REGS; i < PRF_size; ++i) {
        if (PRF[i].free) {
            PRF[i].free = false;
            PRF[i].ready = false;
            *preg_out = (int)i;
            return true;
        }
    }
    return false;
}

static void free_physical_reg(int preg) {
    if (preg >= NUM_REGS) {
        PRF[preg].free = true;
        PRF[preg].ready = false;
    }
}

static void reset_registers() {
    for (size_t i = 0; i < PRF_size; ++i) {
        PRF[i].free = i >= NUM_REGS;
        PRF[i].ready = i < NUM_REGS;
    }
    for (int i = 0; i < NUM_REGS; ++i) RAT[i] = i;
}

// -------------------------------------------------------------------
static uint64_t stage_state_update(procsim_stats_t *stats,
                                   bool *retired_mispredict_out) {
    uint64_t retired_count = 0;
#ifdef DEBUG
    printf("Stage State Update:\n");
#endif
    *retired_mispredict_out = false;
    
    // Retire instructions from the head of the ROB in program order.
    while (ROB_count > 0) {
        ROBEntry *entry = &ROB[ROB_head];
        if (!entry->valid || !entry->completed) {
            // Cannot retire if the head of the ROB is not complete.
            break;
        }
        
        // Retire this instruction.
        retired_count++;
        stats->instructions_retired++;
        
        // If the instruction is a branch, update branch instruction count.
        if (entry->inst->opcode == opcode_t::OPCODE_BRANCH) {
            stats->num_branch_instructions++;
            procsim_driver_update_predictor(entry->inst->pc, entry->inst->br_taken,
                                           entry->inst->dyn_instruction_count);
            // If the branch was mispredicted, update misprediction stats,
            // set the mispredict flag, retire it, and then stop retiring further instructions.
            if (entry->mispredict) {
                stats->branch_mispredictions++;
                *retired_mispredict_out = true;
                // Remove this ROB entry.
                entry->valid = false;
                ROB_head = (ROB_head + 1) % ROB_capacity;
                ROB_count--;
                free(entry->inst);
                reset_registers();
                break;
            }
        }
        
        // For instructions with a destination, free the previous physical register if needed.
        if (entry->dest_preg != -1 && entry->prev_preg >= NUM_REGS) {
            free_physical_reg(entry->prev_preg);
        }
        
        free(entry->inst);
        // Remove the ROB entry by marking it invalid and advancing the head.
        entry->valid = false;
        ROB_head = (ROB_head + 1) % ROB_capacity;
        ROB_count--;
    }
    
    return retired_count;
}


// -------------------------------------------------------------------
// Stage: Execute
static void stage_exec(procsim_stats_t *stats) {
#ifdef DEBUG
    printf("Stage Exec:\n");
    printf("\tProgressing functional units\n");
#endif
    // Iterate over scheduling queue entries
    for (size_t i = 0; i < sched_queue_count; ) {
        ReservationStation *rs = &sched_queue[i];
        if (!rs->valid || !rs->fired) {
            i++;
            continue;
        }
        // If still executing, decrement remaining cycles
        if (rs->remaining_cycles > 0) {
            rs->remaining_cycles--;
        }
        if (rs->remaining_cycles == 0) {
            // Mark instruction as completed:
            // Set destination physical register ready if applicable.
            if (rs->dest_preg != -1) {
                PRF[rs->dest_preg].ready = true;
            }
            // Also mark corresponding ROB entry as complete.
            // For simplicity, search ROB for matching dyncount.
            for (size_t j = 0; j < ROB_capacity; j++) {
                size_t idx = (ROB_head + j) % ROB_capacity;
                if (ROB[idx].valid && ROB[idx].dyn_count == rs->inst->dyn_instruction_count) {
                    ROB[idx].completed = true;
                    break;
                }
            }
            // Remove RS entry by replacing it with the last entry in the queue
            rs->valid = false;
            // Shift remaining entries down
            for (size_t k = i; k < sched_queue_count - 1; k++) {
                sched_queue[k] = sched_queue[k+1];
            }
            sched_queue_count--;
            continue; // do not increment i since we shifted
        }
        i++;
    }
}

// -------------------------------------------------------------------
// Stage: Schedule
static void stage_schedule(procsim_stats_t *stats) {
#ifdef DEBUG
    printf("Stage Schedule:\n");
#endif
    // For functional unit constraints per cycle.
    int alu_fired = 0, mul_fired = 0, lsu_fired = 0;
    int busy_lsus = 0;
    bool store_fired = false;

    /*The LSU is not pipelined, so an executing load continues to occupy
      one LSU until it completes. ALU and MUL issue capacity is handled by
      the per-cycle counters below (the MUL itself is pipelined).
    */
    for (size_t i = 0; i < sched_queue_count; i++) {
        if (sched_queue[i].valid && sched_queue[i].fired &&
            sched_queue[i].fu_type == 2) {
            busy_lsus++;
        }
    }

    // Process RS entries in program order (assumed order in sched_queue)
    for (size_t i = 0; i < sched_queue_count; i++) {
        ReservationStation *rs = &sched_queue[i];
        if (!rs->valid || rs->fired) continue;

        // A result bus marks the producer's PReg ready. Re-read the PRF on
        // every scheduling pass so dependent instructions can wake up.
        if (rs->src1_preg != -1)
            rs->src1_ready = PRF[rs->src1_preg].ready;
        if (rs->src2_preg != -1)
            rs->src2_ready = PRF[rs->src2_preg].ready;

        // Check if source operands are ready
        if (!rs->src1_ready || !rs->src2_ready) continue;
        // For memory operations, perform a simplified memory disambiguation:
        if (rs->fu_type == 2) { // LSU
            // For LOAD, ensure that all preceding STORE instructions (in ROB) are complete.
            if (rs->inst->opcode == opcode_t::OPCODE_LOAD) {
                bool can_fire = true;
                // Scan ROB for any store with lower dyn count that is not complete.
                for (size_t j = 0; j < ROB_capacity; j++) {
                    if (!ROB[j].valid) continue;
                    if (ROB[j].inst->opcode == opcode_t::OPCODE_STORE &&
                        ROB[j].dyn_count < rs->inst->dyn_instruction_count &&
                        !ROB[j].completed) {
                        can_fire = false;
                        break;
                    }
                }
                if (!can_fire)
                    continue;
            }
            // For STORE, wait until all preceding memory instructions (LOAD and STORE) are complete.
            if (rs->inst->opcode == opcode_t::OPCODE_STORE) {
                bool can_fire = true;
                for (size_t j = 0; j < ROB_capacity; j++) {
                    if (!ROB[j].valid) continue;
                    if ((ROB[j].inst->opcode == opcode_t::OPCODE_LOAD ||
                         ROB[j].inst->opcode == opcode_t::OPCODE_STORE) &&
                        ROB[j].dyn_count < rs->inst->dyn_instruction_count &&
                        !ROB[j].completed) {
                        can_fire = false;
                        break;
                    }
                }
                if (!can_fire)
                    continue;
            }
        }
        // Now check functional unit limits based on type:
        if (rs->fu_type == 0) { // ALU (and branch)
            if (alu_fired >= (int)current_conf.num_alu_fus)
                continue;
            alu_fired++;
        } else if (rs->fu_type == 1) { // MUL
            if (mul_fired >= (int)current_conf.num_mul_fus)
                continue;
            mul_fired++;
        } else if (rs->fu_type == 2) { // LSU
            if (busy_lsus + lsu_fired >= (int)current_conf.num_lsu_fus)
                continue;
            // Only allow one store per cycle (if opcode STORE)
            if (rs->inst->opcode == opcode_t::OPCODE_STORE) {
                if (store_fired)
                    continue;
                store_fired = true;
            }
            lsu_fired++;
        }
        // If we reached here, the instruction has acquired a function unit.
        rs->fired = true;
    }
    if (alu_fired + mul_fired + lsu_fired == 0){
        stats->no_fire_cycles++;
    }
}

// -------------------------------------------------------------------
// Stage: Dispatch
static void stage_dispatch(procsim_stats_t *stats) {
#ifdef DEBUG
    printf("Stage Dispatch:\n");
#endif
    uint64_t nop_dispatched = 0, valid_dispatched = 0;
    while (!dispatch_queue.empty()) {
        if (ROB_count == ROB_capacity) {
            stats->rob_no_dispatch_cycles++;
            break;
        }
        if (sched_queue_count == sched_queue_capacity) break;
        if (nop_dispatched == current_conf.dispatch_width ||
            valid_dispatched == current_conf.dispatch_width) break;
        inst_t *inst = dispatch_queue.front();
        if (inst->opcode == 0) {
            dispatch_queue.pop_front();
            free(inst);
            ++nop_dispatched;
            continue;
        }
        int new_preg = -1;
        if (inst->dest != -1 && !allocate_free_reg(&new_preg)) {
            stats->no_dispatch_pregs_cycles++;
            break;
        }
        dispatch_queue.pop_front();
        ++valid_dispatched;

        // Set up a new reservation station (RS) entry.
        ReservationStation new_rs;
        new_rs.valid = true;
        new_rs.fired = false;
        new_rs.inst = inst;
        if (inst->opcode == opcode_t::OPCODE_ADD || inst->opcode == opcode_t::OPCODE_BRANCH) {
            new_rs.fu_type = 0;
            new_rs.remaining_cycles = 1;
        } else if (inst->opcode == opcode_t::OPCODE_MUL) {
            new_rs.fu_type = 1;
            new_rs.remaining_cycles = 3;
        } else if (inst->opcode == opcode_t::OPCODE_LOAD || inst->opcode == opcode_t::OPCODE_STORE) {
            new_rs.fu_type = 2;
            if (inst->opcode == opcode_t::OPCODE_LOAD) {
                if (inst->dcache_hit == cache_lentency_t::CACHE_LATENCY_L1_HIT)
                    new_rs.remaining_cycles = 2;
                else if (inst->dcache_hit == cache_lentency_t::CACHE_LATENCY_L2_HIT)
                    new_rs.remaining_cycles = 10;
                else if (inst->dcache_hit == cache_lentency_t::CACHE_LATENCY_L2_MISS)
                    new_rs.remaining_cycles = 100;
                else
                    new_rs.remaining_cycles = 2;
            } else { // STORE
                new_rs.remaining_cycles = 1;
            }
        } else {
            new_rs.fu_type = 0;
            new_rs.remaining_cycles = 1;
        }
        if (inst->src1 != (int8_t)-1) {
            new_rs.src1_preg = RAT[inst->src1];
            new_rs.src1_ready = PRF[new_rs.src1_preg].ready;
        } else {
            new_rs.src1_preg = -1;
            new_rs.src1_ready = true;
        }
        if (inst->src2 != (int8_t)-1) {
            new_rs.src2_preg = RAT[inst->src2];
            new_rs.src2_ready = PRF[new_rs.src2_preg].ready;
        } else {
            new_rs.src2_preg = -1;
            new_rs.src2_ready = true;
        }

        int old_mapping = -1;
        if (inst->dest != -1) {
            old_mapping = RAT[inst->dest];
            RAT[inst->dest] = new_preg;
            PRF[new_preg].free = false;
            PRF[new_preg].ready = false;
        }

        // Create a new ROB entry for this instruction.
        ROBEntry new_rob;
        new_rob.valid = true;
        new_rob.completed = false;
        new_rob.mispredict = inst->mispredict;
        new_rob.dyn_count = inst->dyn_instruction_count;
        new_rob.dest_preg = (inst->dest == (int8_t)-1) ? -1 : new_preg;
        new_rob.prev_preg = (inst->dest == (int8_t)-1) ? -1 : old_mapping;
        new_rob.inst = inst;
        ROB[ROB_tail] = new_rob;
        ROB_tail = (ROB_tail + 1) % ROB_capacity;
        ROB_count++;

        //Set RS dest_preg to the newly assigned one from free_list.
        new_rs.dest_preg = (inst->dest == (int8_t)-1) ? -1 : new_preg;
        sched_queue[sched_queue_count++] = new_rs;
        // stats->instructions_fetched++;
    }
}


// -------------------------------------------------------------------
// Stage: Fetch
static void stage_fetch(procsim_stats_t *stats) {
#ifdef DEBUG
    printf("Stage Fetch:\n");
#endif
    for (uint64_t i = 0; i < current_conf.fetch_width; ++i) {
        driver_read_status_t drs;
        const inst_t *fetched = procsim_driver_read_inst(&drs);
        if (fetched) {
            inst_t *copy = (inst_t *)malloc(sizeof(inst_t));
            *copy = *fetched;
            dispatch_queue.push_back(copy);
            stats->instructions_fetched++;
        } else if (drs == DRIVER_READ_ICACHE_MISS) {
            dispatch_queue.push_back((inst_t *)calloc(1, sizeof(inst_t)));
        } else if (drs == DRIVER_READ_MISPRED) {
            // Wrong-path NOPs count as fetched but never enter the queue.
            stats->instructions_fetched++;
        }
    }
}

// -------------------------------------------------------------------
// Initialization function for the processor simulator.
void procsim_init(const procsim_conf_t *sim_conf, procsim_stats_t *stats) {
    // Copy configuration for global use.
    current_conf = *sim_conf;
    current_stats = stats;
    
    dispatch_queue.clear();

    // Allocate scheduling queue capacity = s * (a + m + l)
    sched_queue_capacity = sim_conf->num_schedq_entries_per_fu * (sim_conf->num_alu_fus + sim_conf->num_mul_fus + sim_conf->num_lsu_fus);
    sched_queue_count = 0;
    sched_queue = (ReservationStation *)malloc(sched_queue_capacity * sizeof(ReservationStation));
    
    // Allocate ROB with capacity defined in config.
    ROB_capacity = sim_conf->num_rob_entries;
    ROB_count = 0;
    ROB_head = ROB_tail = 0;
    ROB = (ROBEntry *)malloc(ROB_capacity * sizeof(ROBEntry));
    memset(ROB, 0, ROB_capacity * sizeof(ROBEntry));
    
    // Setup physical register file: size = NUM_REGS + num_pregs.
    PRF_size = NUM_REGS + sim_conf->num_pregs;
    PRF = (PRFEntry *)malloc(PRF_size * sizeof(PRFEntry));
    // Architectural registers (first NUM_REGS): allocated and ready.
    for (size_t i = 0; i < NUM_REGS; i++) {
        PRF[i].free = false;
        PRF[i].ready = true;
    }
    // The rest are free and not ready.
    for (size_t i = NUM_REGS; i < PRF_size; i++) {
        PRF[i].free = true;
        PRF[i].ready = false;
    }
    
    // Initialize RAT to identity mapping.
    for (int r = 0; r < NUM_REGS; r++) {
        RAT[r] = r;
    }
    
#ifdef DEBUG
    printf("\nScheduling queue capacity: %lu instructions\n", sched_queue_capacity);
    printf("Initial RAT state:\n");
    print_rat();
    printf("\n");
#endif
}

// -------------------------------------------------------------------
// Main cycle function: procsim_do_cycle() is provided but calls our stage functions.
uint64_t procsim_do_cycle(procsim_stats_t *stats,
                          bool *retired_mispredict_out) {
#ifdef DEBUG
    printf("================================ Begin cycle %" PRIu64 " ================================\n", stats->cycles);
#endif

    uint64_t retired_this_cycle = stage_state_update(stats, retired_mispredict_out);
    if (*retired_mispredict_out) {
#ifdef DEBUG
        printf("%" PRIu64 " instructions retired (mispredict). Not proceeding with other stages.\n", retired_this_cycle);
#endif
    } else {
#ifdef DEBUG
        printf("%" PRIu64 " instructions retired. Proceeding with pipeline stages.\n", retired_this_cycle);
#endif
        stage_exec(stats);
        stage_schedule(stats);
        stage_dispatch(stats);
        stage_fetch(stats);
        
    }
    
    // Update max and avg usage stats (simplified update using current counts)
    if (dispatch_queue.size() > stats->dispq_max_usage)
        stats->dispq_max_usage = dispatch_queue.size();
    stats->dispq_avg_usage = ((stats->dispq_avg_usage * stats->cycles) + dispatch_queue.size()) / (stats->cycles + 1);
    
    if (sched_queue_count > stats->schedq_max_usage)
        stats->schedq_max_usage = sched_queue_count;
    stats->schedq_avg_usage = ((stats->schedq_avg_usage * stats->cycles) + sched_queue_count) / (stats->cycles + 1);
    
    if (ROB_count > stats->rob_max_usage)
        stats->rob_max_usage = ROB_count;
    stats->rob_avg_usage = ((stats->rob_avg_usage * stats->cycles) + ROB_count) / (stats->cycles + 1);
    
    stats->cycles++;
#ifdef DEBUG
    printf("End-of-cycle dispatch queue usage: %lu\n", dispatch_queue.size());
    printf("End-of-cycle sched queue usage: %lu\n", sched_queue_count);
    printf("End-of-cycle ROB usage: %lu\n", ROB_count);
    printf("================================ End cycle %" PRIu64 " ================================\n", stats->cycles);
    print_rat();
    print_prf();
    print_rob();
#endif
    return retired_this_cycle;
}

// -------------------------------------------------------------------
// Finalize the simulation and free memory.
void procsim_finish(procsim_stats_t *stats) {
    if (stats->cycles > 0)
        stats->ipc = (double)stats->instructions_retired / stats->cycles;
    else
        stats->ipc = 0.0;

    for (inst_t *inst : dispatch_queue) free(inst);
    dispatch_queue.clear();

    // Free scheduling queue.
    free(sched_queue);
    sched_queue = NULL;
    
    // Free ROB.
    free(ROB);
    ROB = NULL;
    
    // Free physical register file.
    free(PRF);
    PRF = NULL;
}
