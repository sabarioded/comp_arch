/* 046267 Computer Architecture - Winter 20/21 - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>

#define IDLE -1

typedef struct {
	tcontext **context_s;
	int *threads_line;
	int thread_num;
	int load_latency;
	int store_latency;
	int context_switch_latency;
	int how_many_threads_done;
	Instruction *curr_inst;
	int cycle_counter;
	int instruction_counter;
	int *threads_busy; //each spot holds for how many cycles each thread is busy

} core_sim;

//global 
core_sim* core_sim_FG;
core_sim* core_sim_BLOCKED;

/**
 * @brief constructor for a thread's context
 * @return tcontext* a new tcontext object
 */
tcontext* context_init() {
	tcontext* new_context = (tcontext*)malloc(sizeof(tcontext));
	if(new_context == NULL)
		return NULL;
	for(int i = 0; i < REGS_COUNT; i++){
		new_context->reg[i] = 0;
	}
	return new_context;
}


/**
 * @brief constructor for a multithreading simulator
 * @return core_sim* a new simulator object
 */
core_sim* core_sim_init() {
	core_sim* new_core = (core_sim*)malloc(sizeof(core_sim));
	if(new_core == NULL)
		return NULL;
	new_core->curr_inst = (Instruction*)malloc(sizeof(Instruction));
	new_core->thread_num = SIM_GetThreadsNum();
	new_core->load_latency = SIM_GetLoadLat();
	new_core->store_latency = SIM_GetStoreLat();
	new_core->context_switch_latency = SIM_GetSwitchCycles();
	new_core->how_many_threads_done = 0;
	new_core->cycle_counter = 0;
	new_core->instruction_counter = 0;
	new_core->threads_line = (int*)malloc(sizeof(int)*new_core->thread_num);
	if(new_core->threads_line == NULL)
		return NULL;
	for(int i = 0; i < new_core->thread_num; i++) {
		new_core->threads_line[i] = 0;
	}
	new_core->context_s = (tcontext**)malloc(sizeof(tcontext*)*new_core->thread_num);
	if(new_core->context_s == NULL)
		return NULL;
	for(int i = 0; i < new_core->thread_num; i++) {
		new_core->context_s[i] = context_init();
	}
	new_core->threads_busy = (int*)malloc(sizeof(int)*new_core->thread_num);
	if(new_core->threads_busy == NULL)
		return NULL;
	for(int i = 0; i < new_core->thread_num; i++) {
		new_core->threads_busy[i] = 0;
	}
	return new_core;
}
/**
 * @brief destroy a tcontext object
 * @param context The tcontext to destroy
 */
void context_destroy(tcontext* context) {
	free(context);
}
/**
 * @brief destroy a multithreading simulator
 * @param core_sim The simulator to destroy
 */
void core_sim_destroy(core_sim* core_sim) {
	for(int i = 0; i < core_sim->thread_num; i++) {
		context_destroy(core_sim->context_s[i]);
	}
	free(core_sim->threads_busy);
	free(core_sim->threads_line);
	free(core_sim);
}
/**
 * @brief updates threads_busy array
 * @param core_sim The simulator
 */
void update_busy_status(core_sim* core_sim,int cycles_passed) {
	for(int i = 0; i < core_sim->thread_num; i++) {
		core_sim->threads_busy[i] =((core_sim->threads_busy[i] - cycles_passed) >= 0) ? (core_sim->threads_busy[i] - cycles_passed) : 0;
	}
}
/**
 * @brief finds an available thread to run according to RR policy
 * @param core_sim The simulator
 * @return an available thread index
 */
int get_next_free_thread(core_sim* core_sim, int curr_thread,bool BLOCKED){
	if(core_sim->threads_busy[curr_thread] && BLOCKED)
		return curr_thread;
	int index = (curr_thread == (core_sim->thread_num-1)) ? 0 : (curr_thread+1);
	while(core_sim->threads_busy[index] != 0 ) {
	  	if(index == curr_thread)
			return IDLE;
		if(index >= core_sim->thread_num)
			index = 0;
		index++;
	}
	return index;
	
}

/**
 * @brief handle load command
 * @param core_sim The simulator
 * @param curr_thread The current thread
 */
void perform_load(core_sim* core_sim,int curr_thread) {
	int reg1 = core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->src1_index];
	int reg2 = core_sim->curr_inst->src2_index_imm;
	if(!core_sim->curr_inst->isSrc2Imm)
		reg2 = core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->src2_index_imm];
	
	int mem_adress = (reg1 + reg2) - ((reg1 + reg2) % 4);
	int32_t dst_reg = (int32_t)core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->dst_index];
	SIM_MemDataRead(mem_adress, &dst_reg);
	core_sim->threads_busy[curr_thread] = core_sim->load_latency;

}
/**
 * @brief handle store command
 * @param core_sim The simulator
 * @param curr_thread The current thread
 */
void perform_store(core_sim* core_sim,int curr_thread) {
	int dst = core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->dst_index];
	int reg1 = core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->src1_index];
	int reg2 = core_sim->curr_inst->src2_index_imm;
	if(!core_sim->curr_inst->isSrc2Imm)
		reg2 = core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->src2_index_imm];
	
	int mem_adress = (dst + reg2) - ((dst + reg2) % 4);
	SIM_MemDataWrite(mem_adress,reg1);
	core_sim->threads_busy[curr_thread] = core_sim->store_latency;
}
/**
 * @brief handle arithmetic command (add/addi/sub/subi/nop)
 * @param core_sim The simulator
 * @param curr_thread The current thread
 * @param opcode The command's opcode
 */
void perform_arithmetic_command(core_sim* core_sim,int curr_thread,int opcode) {
	int reg1 = core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->src1_index];
	int reg2 = core_sim->curr_inst->src2_index_imm;
	if(!core_sim->curr_inst->isSrc2Imm)
		reg2 = core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->src2_index_imm];
	if((opcode == CMD_ADD) || (opcode == CMD_ADDI)) {
		core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->dst_index] = reg1 + reg2;
	}
	if((opcode == CMD_SUB) || (opcode == CMD_SUBI)) {
		core_sim->context_s[curr_thread]->reg[core_sim->curr_inst->dst_index] = reg1 - reg2;
	}
}

void CORE_BlockedMT() {
	int curr_thread = 0;
	int opcode = 0;
	int old_thread;
	int cycles_passed = 1;
	core_sim_BLOCKED = core_sim_init();
	if(core_sim_BLOCKED == NULL)
		return;
	while(core_sim_BLOCKED->how_many_threads_done < core_sim_BLOCKED->thread_num) {
		/* get next free thread and update cycles accordingly */
		old_thread = curr_thread;
		curr_thread = get_next_free_thread(core_sim_BLOCKED,curr_thread,true /* blocked */);
		if(curr_thread != old_thread) {
			cycles_passed = core_sim_BLOCKED->context_switch_latency;
		} 
		else {
			cycles_passed = 1;
		} 
		update_busy_status(core_sim_BLOCKED,cycles_passed);
		core_sim_BLOCKED->cycle_counter += cycles_passed;
		if(curr_thread == IDLE) {
			continue; //if idle move to next cycle
		}
		/* read next instruction and update instruction counter*/
		SIM_MemInstRead(core_sim_BLOCKED->threads_line[curr_thread],core_sim_BLOCKED->curr_inst,curr_thread);
		core_sim_BLOCKED->instruction_counter++;
		opcode = (int)core_sim_BLOCKED->curr_inst->opcode;
		/* update registers according to command type */
		if(opcode == CMD_HALT) {
			core_sim_BLOCKED->how_many_threads_done++;
			core_sim_BLOCKED->threads_busy[curr_thread] = -1;//DONE
			curr_thread++;
			continue;
		}
		if(opcode == CMD_LOAD){
			perform_load(core_sim_BLOCKED,curr_thread);
			continue;
		}
		if(opcode == CMD_STORE){
			perform_store(core_sim_BLOCKED,curr_thread);
			continue;
		}
		/* else other commands */
		perform_arithmetic_command(core_sim_BLOCKED,curr_thread,opcode);
	}

}

void CORE_FinegrainedMT() {
	int curr_thread = 0;
	int opcode = 0;
	int old_thread;
	int cycles_passed = 1;
	core_sim_FG = core_sim_init();
	if(core_sim_FG == NULL)
		return;
	while(core_sim_FG->how_many_threads_done < core_sim_FG->thread_num) {
		/* get next free thread and update cycles accordingly */
		curr_thread = get_next_free_thread(core_sim_FG,curr_thread,false/* not blocked */);
		update_busy_status(core_sim_FG,cycles_passed);
		core_sim_FG->cycle_counter += cycles_passed;
		if(curr_thread == IDLE) {
			continue; //if idle move to next cycle
		}
		/* read next instruction and update instruction counter*/
		SIM_MemInstRead(core_sim_FG->threads_line[curr_thread],core_sim_FG->curr_inst,curr_thread);
		core_sim_FG->instruction_counter++;
		opcode = (int)core_sim_FG->curr_inst->opcode;
		/* update registers according to command type */
		if(opcode == CMD_HALT) {
			core_sim_FG->how_many_threads_done++;
			core_sim_FG->threads_busy[curr_thread] = -1;//DONE
			curr_thread++;
			continue;
		}
		if(opcode == CMD_LOAD){
			perform_load(core_sim_FG,curr_thread);
			continue;
		}
		if(opcode == CMD_STORE){
			perform_store(core_sim_FG,curr_thread);
			continue;
		}
		/* else other commands */
		perform_arithmetic_command(core_sim_FG,curr_thread,opcode);
}

double CORE_BlockedMT_CPI(){
	if(core_sim_BLOCKED == NULL)
		return 0;
	double CPI = (double)core_sim_BLOCKED->cycle_counter/core_sim_BLOCKED->instruction_counter;
	core_sim_destroy(core_sim_BLOCKED);
	return CPI;
}

double CORE_FinegrainedMT_CPI(){
	if(core_sim_FG == NULL)
		return 0;
	double CPI = (double)core_sim_FG->cycle_counter/core_sim_FG->instruction_counter;
	core_sim_destroy(core_sim_FG);
	return CPI;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
	for(int i = 0; i < REGS_COUNT;i++) {
		context->reg[i] = core_sim_BLOCKED->context_s[threadid]->reg[i];
	}
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
	for(int i = 0; i < REGS_COUNT;i++) {
		context->reg[i] = core_sim_FG->context_s[threadid]->reg[i];
	}
}
