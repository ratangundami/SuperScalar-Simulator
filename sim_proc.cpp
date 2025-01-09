#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <cmath>
#include <string.h>
#include <algorithm>
#include <iomanip>
#include "sim_proc.h"
#include <vector>


using namespace std;

// Global Variables
uint32_t dynamic_instr_count = 0;
uint32_t total_cycles = 0;
uint32_t total_ipc = 0;

uint32_t seq_no = 0;
uint32_t cycle_count = 0;


bool execution_remaining = true;
bool file_fetch_remaining = true;
bool DEBUG_PRINT = false;


// Global declarations
class ReorderBuffer;
class ReorderBufferBlock;
class Pipeline;

class ReorderBufferBlock{
public:
   reorder_buffer_struct data;
   unsigned int rob_tag;
   ReorderBufferBlock* next;
   ReorderBufferBlock* prev;

   ReorderBufferBlock(reorder_buffer_struct& data, int name){
      this->data = data;
      this->rob_tag = name;
      this->next = nullptr;
      this->prev = nullptr;
   }
};

class ReorderBuffer{
public:
   ReorderBufferBlock* head;
   ReorderBufferBlock* tail;
   int rob_size;
   int current_rob_size;

   ReorderBuffer(){
      this->head = nullptr;
      this->tail = nullptr;
      current_rob_size = 0;
      rob_size = 0;
   }

   ~ReorderBuffer() {
        if (!head) return;

        ReorderBufferBlock* current = head;
        do {
            ReorderBufferBlock* temp = current;
            current = current->next;
            delete temp;
        } while (current != head);

        head = nullptr;
        tail = nullptr;
   }

    void initialize(int robSize) {
        rob_size = robSize;
        if (rob_size <= 0) return;

        for (int i = 0; i < rob_size; ++i) {
            reorder_buffer_struct initData = {-1, 0, 0, 0}; // Initialize block data
            ReorderBufferBlock* newBlock = new ReorderBufferBlock(initData, 1000+i);
            if (head == nullptr) {
                // First block becomes head and tail
                head = tail = newBlock;
                head->next = head->prev = head; // Circular link
            } 
            else {
                // Add new block to the end of the list
                tail->next = newBlock;
                newBlock->prev = tail;
                newBlock->next = head; // Circular link
                head->prev = newBlock;
                tail = newBlock; // Update tail
            }
        }

        // make head and tail pointing to the first element to begin with
        tail = head;

    }

   void displayROB(){
        if (!head) {
            cout << "Reorder Buffer is empty." << endl;
            return;
        }

        ReorderBufferBlock* current = head;
        do {
            cout << "  ROB " << current->rob_tag
                << ": [dst=" << current->data.dst
                << ", rdy=" << current->data.rdy
                << ", mis=" << current->data.mis
                << ", pc= " << hex << current->data.pc << dec << "]" << endl;
            current = current->next;
        } while (current != head);
        cout << "   current size = " << current_rob_size << endl;
   }

    int modifyTail(int dst, unsigned int rdy, unsigned int mis, unsigned int pc) {
        int return_rob_tag;

        // Modify the fields of the tail node
        tail->data.dst = dst;
        tail->data.rdy = rdy;
        tail->data.mis = mis;
        tail->data.pc = pc;
        return_rob_tag = tail->rob_tag;

        // Move the tail pointer to the next node
        tail = tail->next;
        current_rob_size++;
        return return_rob_tag;
    }

    int getEmptySlots(){
        return rob_size-current_rob_size;
    }

    void modifyHead() {
        head = head->next; 
        current_rob_size--;
    }

    ReorderBufferBlock* findTheBlock(unsigned int name){
        if (head == nullptr) { // List is empty
            return nullptr;
        }

        ReorderBufferBlock* temp = head;
        do
        {
            if(temp->rob_tag == name){
                return temp;
            }
            temp = temp->next;
        } while (temp != head);

        return nullptr; // No match found (never happens)
    }
};


class Pipeline {
public:
    instruction_format* Decode_Register;
    instruction_format* Rename_Register;
    instruction_format* RegRead_Register;
    instruction_format* Dispatch_Register;

    int rob_size;
    int iq_size;
    int pipe_width;
    int register_count = 67;

    ReorderBuffer reorder_buffer;
    issue_queue_struct* issue_queue;
    rename_map_table_struct rename_map_table[67]; 
    vector<instruction_format> execute_list;
    vector<instruction_format> Writeback_Register;
    vector<instruction_format> Retire_Register;

    Pipeline(int ROBSize, int IQSize, int Width){
        rob_size = ROBSize;
        iq_size = IQSize;
        pipe_width = Width;

        // Initialize ROB
        reorder_buffer.initialize(rob_size);

        if(DEBUG_PRINT == true){
            printf("=== Empty Reorder Buffer =========\n");
            reorder_buffer.displayROB();
        }

        // Initialize RMT
        for(int i=0; i<register_count; i++){
            rename_map_table[i].valid_bit = 0;
            rename_map_table[i].rob_tag = -1;
        }

        if(DEBUG_PRINT == true){
            printf("=== Empty Rename Map Table =========\n");
            displayRMT();
        }

        // Initialize Issue Queue
        issue_queue = new issue_queue_struct[iq_size];

        // Initialize Bundles
        Decode_Register = new instruction_format[pipe_width];
        Rename_Register = new instruction_format[pipe_width];
        RegRead_Register = new instruction_format[pipe_width];
        Dispatch_Register = new instruction_format[pipe_width];  
    }

    ~Pipeline() {
        delete Decode_Register;
        delete Rename_Register;
        delete RegRead_Register;
        delete Dispatch_Register; 
        delete[] issue_queue;
    }

    void clearRegister(instruction_format* Register_t){
        for(int i = 0; i<pipe_width; i++){
            Register_t[i].sequence_id = -1;
            Register_t[i].pc = -1;
            Register_t[i].op_type = -1;
            Register_t[i].src1 = -1;
            Register_t[i].src1_org = -1;
            Register_t[i].src1_rdy = 0;
            Register_t[i].src2 = -1;
            Register_t[i].src2_org = -1;
            Register_t[i].src2_rdy = 0;
            Register_t[i].dst = -1;
            Register_t[i].fetch = {0,0};
            Register_t[i].decode = {0,0};
            Register_t[i].rename = {0,0};
            Register_t[i].reg_read = {0,0};
            Register_t[i].dispatch = {0,0};
            Register_t[i].issue_queue = {0,0};
            Register_t[i].execute = {0,0};
            Register_t[i].writeback = {0,0};
            Register_t[i].retire = {0,0};
        }
    }

    // void copyRegister(instruction_format* src_reg, instruction_format* dst_reg){
    //     for(int i=0; i<pipe_width; i++){
    //         dst_reg[i] = src_reg[i];
    //     }
    // }

    // Display Function to display the instruction at the end of the cycle
    void display_instruction(instruction_format print_instr) {
        printf("%ld fu{%d} src{%d,%d} dst{%d}", print_instr.sequence_id, print_instr.op_type, print_instr.src1, print_instr.src2, print_instr.dst);
        printf(" FE{%ld,%d}", print_instr.fetch.begin_cycle, print_instr.fetch.duration);
        printf(" DE{%ld,%d}", print_instr.decode.begin_cycle, print_instr.decode.duration);
        printf(" RN{%ld,%d}", print_instr.rename.begin_cycle, print_instr.rename.duration);
        printf(" RR{%ld,%d}", print_instr.reg_read.begin_cycle, print_instr.reg_read.duration);
        printf(" DI{%ld,%d}", print_instr.dispatch.begin_cycle, print_instr.dispatch.duration);
        printf(" IS{%ld,%d}", print_instr.issue_queue.begin_cycle, print_instr.issue_queue.duration);
        printf(" EX{%ld,%d}", print_instr.execute.begin_cycle, print_instr.execute.duration);
        printf(" WB{%ld,%d}", print_instr.writeback.begin_cycle, print_instr.writeback.duration);
        printf(" RT{%ld,%d}\n", print_instr.retire.begin_cycle, print_instr.retire.duration);
    }

    // Check if the register passed is empty
    bool RegisterEmpty(instruction_format* Register_t){
        if(Register_t[0].sequence_id == -1){
            return true;
        }
        else{
            return false;
        }
    }

    // get the width or number of instructions in the register
    int GetBundleSize(instruction_format* Register_t){
        int bundle_size = 0;
        for(int i=0; i<pipe_width; i++){
            if(Register_t[i].sequence_id != -1){
                bundle_size++;
            }
        }
        return bundle_size;
    }

    // display RMT
    void displayRMT(){
        for(int i=0; i<register_count; i++){
            printf("reg%d: Valid=%d, ROB_TAG=%d\n", i, rename_map_table[i].valid_bit, rename_map_table[i].rob_tag);
        }
    }

    // Display Issue Queue
    void displayIQ(){
        for(int i=0; i<iq_size; i++){
            printf("IQ %d: Valid=%d, SeqNo=%ld, dst=%d ", i, issue_queue[i].valid_bit, issue_queue[i].sequence_id, issue_queue[i].dst_tag);
            printf("rs1_rdy=%d, rs1_tag=%d, rs2_rdy=%d, rs2_tag=%d\n", issue_queue[i].rs1_rdy, issue_queue[i].rs1_tag, issue_queue[i].rs2_rdy, issue_queue[i].rs2_tag);
        }
    }

    // Check the number of free entries in IQ
    int checkIQFreeSize(){
        int free_entries = 0;
        for(int i=0; i<iq_size; i++){
            if(issue_queue[i].valid_bit == 0){
                free_entries++;
            }
        }
        return free_entries;
    }

    // Add the entry to Issue Queue. Add to the nearest invalid free entry
    void AddIssueToIQ(long int sequence_id, int dst, unsigned int rs1_rdy, int rs1_tag, unsigned int rs2_rdy, int rs2_tag, instruction_format instr_in_iq){
        for(int i=0; i<iq_size; i++){
            if(issue_queue[i].valid_bit == 0){
                issue_queue[i].valid_bit = 1;
                issue_queue[i].sequence_id = sequence_id;
                issue_queue[i].dst_tag = dst;
                issue_queue[i].rs1_rdy = rs1_rdy; 
                issue_queue[i].rs1_tag = rs1_tag; 
                issue_queue[i].rs2_rdy = rs2_rdy;
                issue_queue[i].rs2_tag = rs2_tag; 
                issue_queue[i].instr_in_iq = instr_in_iq;
                break; // break after adding
            }
        }
    }

    // Remove the entry from the Issue Queue
    void RemoveIssueFromIQ(long int sequence_id){
        for(int i=0; i<iq_size; i++){
            if(issue_queue[i].sequence_id == sequence_id){
                issue_queue[i].valid_bit = 0;
                issue_queue[i].sequence_id = -1;
                issue_queue[i].dst_tag = -1;
                issue_queue[i].rs1_rdy = 0; 
                issue_queue[i].rs1_tag = 0;
                issue_queue[i].rs2_rdy = 0;
                issue_queue[i].rs2_tag = 0;
            }
        }
    }

    // Update the entry in the Issue Queue
    void UpdateIssueInIQ(long int sequence_id, int dst, unsigned int rs1_rdy, int rs1_tag, unsigned int rs2_rdy, int rs2_tag){
        for(int i=0; i<iq_size; i++){
            if(issue_queue[i].sequence_id == sequence_id && issue_queue[i].valid_bit == 1){
                issue_queue[i].dst_tag = dst;
                issue_queue[i].rs1_rdy = rs1_rdy; 
                issue_queue[i].rs1_tag = rs1_tag; 
                issue_queue[i].rs2_rdy = rs2_rdy;
                issue_queue[i].rs2_tag = rs2_tag; 
            }
        }
    }

    // Pipeline Stage Fetch Handling
    void Fetch(FILE *FP){
        int op_type, dest, src1, src2;  // Variables are read from trace file
        uint64_t pc; // Variable holds the pc read from input file
        if(RegisterEmpty(Decode_Register) == true){
            for(int i=0; i<pipe_width; i++){
                if(fscanf(FP, "%lx %d %d %d %d", &pc, &op_type, &dest, &src1, &src2) != EOF){
                    if(DEBUG_PRINT == true){
                        printf("%lx %d %d %d %d\n", pc, op_type, dest, src1, src2);  //Print to check if inputs have been read correctly
                    }
                    file_fetch_remaining = true;
                    // Initialize the instruction
                    instruction_format new_instr;
                    new_instr.sequence_id = seq_no;
                    new_instr.pc = pc;
                    new_instr.op_type = op_type;
                    new_instr.src1 = src1;
                    new_instr.src1_org = src1;
                    new_instr.src2 = src2;
                    new_instr.src2_org = src2;
                    new_instr.dst = dest;

                    // start the begin cycle and duration
                    new_instr.fetch.begin_cycle = cycle_count;
                    new_instr.fetch.duration++;
                    // Push the details to DE Register
                    Decode_Register[i] = new_instr;
                    
                    seq_no++;
                }
                else{
                    file_fetch_remaining = false;
                }
            }
        }

        // debug print
        if(DEBUG_PRINT == true){
            printf("FETCH. DE Register\n");
            for(int i=0; i<pipe_width; i++){
                display_instruction(Decode_Register[i]);
            }
            printf("\n");
        }

    }

    void Decode(){
        // iterate through the Decode Register and increment the duration
        for(int i=0; i<pipe_width; i++){
            if(Decode_Register[i].sequence_id != -1){
                Decode_Register[i].decode.duration++;
                if(Decode_Register[i].decode.begin_cycle == 0){
                    Decode_Register[i].decode.begin_cycle = cycle_count;
                }
            }
        }

        if(RegisterEmpty(Decode_Register) == false){ // 
            if(RegisterEmpty(Rename_Register) == true){
                for(int i=0; i<pipe_width; i++){
                    Rename_Register[i] = Decode_Register[i]; // Copy the Decode register to Rename Register
                }
                clearRegister(Decode_Register); // clear Decode Register for next bundle to come
            }
        }

        if(DEBUG_PRINT == true){
            printf("DECODE. RN Register\n");
            for(int i=0; i<pipe_width; i++){
                display_instruction(Rename_Register[i]);
            }
            printf("\n");
        }

    }

    void Rename(){
        // iterate through the Rename Register and increment the duration
        for(int i=0; i<pipe_width; i++){
            if(Rename_Register[i].sequence_id != -1){
                Rename_Register[i].rename.duration++;
                if(Rename_Register[i].rename.begin_cycle == 0){
                    Rename_Register[i].rename.begin_cycle = cycle_count;
                }
            }
        }

        if(RegisterEmpty(Rename_Register) == false){ // RN Register has a bundle
            if((RegisterEmpty(RegRead_Register) == true) &&  (reorder_buffer.getEmptySlots() >= pipe_width)){ // RR is empty and ROB has enough space

                // ROB Process and RN to RR copy
                for(int i=0; i<pipe_width; i++){
                    if(Rename_Register[i].sequence_id > -1){
                        // Step 1: allocate an entry in the ROB for the instruction and update the rename table
                        int rob_tag = reorder_buffer.modifyTail(Rename_Register[i].dst, 0, 0, Rename_Register[i].pc);

                        // Step 2:  rename its source registers
                        // src 1
                        if(Rename_Register[i].src1 >= 0 && rename_map_table[Rename_Register[i].src1].valid_bit == 1){
                            Rename_Register[i].src1 = rename_map_table[Rename_Register[i].src1].rob_tag;
                        }
                        // src 2
                        if(Rename_Register[i].src2 >= 0 && rename_map_table[Rename_Register[i].src2].valid_bit == 1){
                            Rename_Register[i].src2 = rename_map_table[Rename_Register[i].src2].rob_tag;
                        }

                        // Step 3:  rename its dst registers. Always assigned a rob_tag
                        int prev_dst = Rename_Register[i].dst;
                        Rename_Register[i].dst = rob_tag;

                        // Step 4: Update the RMT at the End
                        if(prev_dst >= 0){
                            rename_map_table[prev_dst].valid_bit = 1;
                            rename_map_table[prev_dst].rob_tag = rob_tag;
                        }
                        // Copy the RN to RR
                        RegRead_Register[i] = Rename_Register[i]; // Copy the Rename register to RegRead Register
                    }
                }

                clearRegister(Rename_Register); // clear Decode Register for next bundle to come
            }
        }

        if(DEBUG_PRINT == true){
            printf("RENAME. RR Register\n");
            for(int i=0; i<pipe_width; i++){
                display_instruction(RegRead_Register[i]);
            }
            printf("\n");

            printf("=== Reorder Buffer Cycle (%u) =========\n", cycle_count);
            reorder_buffer.displayROB();

            printf("=== Rename Map Table Cycle (%u) =========\n", cycle_count);
            displayRMT();
        }
    }

    void RegRead(){
        // iterate through the RegRead Register and increment the duration
        for(int i=0; i<pipe_width; i++){
            if(RegRead_Register[i].sequence_id != -1){
                RegRead_Register[i].reg_read.duration++;
                if(RegRead_Register[i].reg_read.begin_cycle == 0){
                    RegRead_Register[i].reg_read.begin_cycle = cycle_count;
                }
            }
        }

        if(RegisterEmpty(RegRead_Register) == false){ // if there is bundle in reg_read
            if(RegisterEmpty(Dispatch_Register) == true){
                // Process the RR bundle (Will be implemented after WB)
                ReorderBufferBlock* match;
                for(int i=0; i < pipe_width; i++){
                    // Find the matching ROB element for src1
                    if(RegRead_Register[i].src1 >= 1000){
                        match = reorder_buffer.findTheBlock(RegRead_Register[i].src1);
                        if(match->data.rdy == 1){
                            RegRead_Register[i].src1_rdy = 1;
                        }
                    }

                    // Find the matching ROB element for src2
                    if(RegRead_Register[i].src2 >= 1000){
                        match = reorder_buffer.findTheBlock(RegRead_Register[i].src2);
                        if(match->data.rdy == 1){
                            RegRead_Register[i].src2_rdy = 1;
                        }
                    }
                }

                // Advance the data to DI Register
                for(int i=0; i<pipe_width; i++){
                    Dispatch_Register[i] = RegRead_Register[i]; // Copy the RR register to DI Register
                }
                clearRegister(RegRead_Register); // clear RR Register for next bundle to come
            }
        }

        if(DEBUG_PRINT == true){
            printf("REG_READ. DI Register\n");
            for(int i=0; i<pipe_width; i++){
                display_instruction(Dispatch_Register[i]);
            }
            printf("\n");
        }

    }

    void Dispatch(){
        // iterate through the Dispatch Register and increment the duration
        for(int i=0; i<pipe_width; i++){
            if(Dispatch_Register[i].sequence_id != -1){
                Dispatch_Register[i].dispatch.duration++;
                if(Dispatch_Register[i].dispatch.begin_cycle == 0){
                    Dispatch_Register[i].dispatch.begin_cycle = cycle_count;
                }
            }
        }
        if(RegisterEmpty(Dispatch_Register) == false){ // DI contains a bundle
            int bundle_size = GetBundleSize(Dispatch_Register);
            if(checkIQFreeSize() >= bundle_size){ // real condition
            // if(checkIQFreeSize() >= pipe_width){ // temp condition
                int rs1_rdy = 0;
                int rs1_tag = 0;
                int rs2_rdy = 0;
                int rs2_tag = 0;
                for(int i=0; i<bundle_size; i++){
                    // Look for a rob tag if present and add that rdy bit
                    rs1_tag = Dispatch_Register[i].src1;
                    rs2_tag = Dispatch_Register[i].src2;

                    ReorderBufferBlock* match;
                    if(Dispatch_Register[i].src1 >= 1000){ // condition to check if the src1 is ROB tag or ARF reg
                        match = reorder_buffer.findTheBlock(Dispatch_Register[i].src1);
                        rs1_rdy = match->data.rdy;
                        if(Dispatch_Register[i].src1_rdy == 1){ // Check for the EX Wakeup
                            rs1_rdy = 1;
                        }
                    }
                    else{
                        rs1_rdy = 1; // RDY is one if its not a ROB Tag
                    }

                    if(Dispatch_Register[i].src2 >= 1000){ // condition to check if the src2 is ROB tag or ARF reg
                        match = reorder_buffer.findTheBlock(Dispatch_Register[i].src2);
                        rs2_rdy = match->data.rdy;
                        if(Dispatch_Register[i].src2_rdy == 1){ // Check for the EX Wakeup
                            rs2_rdy = 1;
                        }
                    }
                    else{
                        rs2_rdy = 1; // RDY is one if its not a ROB Tag
                    }
                    AddIssueToIQ(Dispatch_Register[i].sequence_id, Dispatch_Register[i].dst, rs1_rdy, rs1_tag, rs2_rdy, rs2_tag, Dispatch_Register[i]);

                }
                clearRegister(Dispatch_Register); // clear Decode Register for next bundle to come            

            }
        }

        if(DEBUG_PRINT == true){
            printf("DISPATCH. ISSUE QUEUE\n");
            displayIQ();
            printf("\n");
        }

    }

    void Issue(){
        int remaining_issues = iq_size - checkIQFreeSize();
        if(remaining_issues > 0){ // Check if Issue Queue has instructions
            // Sort the Issue Queue based on sequence_id
            sort(issue_queue, issue_queue + iq_size, [](const issue_queue_struct& A, const issue_queue_struct& B){
                if (A.valid_bit == 0 && B.valid_bit == 0) {
                    return false; // Both invalid, preserve order
                } else if (A.valid_bit == 0) {
                    return false; // Invalid entries go to the end
                } else if (B.valid_bit == 0) {
                    return true; // Valid entries come before invalid
                }
                return A.sequence_id < B.sequence_id;
            });
            // Increment the duration of all the instruction present inside Issue queue
            for(int i=0; i < iq_size; i++){
                // Check the valid instructions in Issue Queue
                if(issue_queue[i].valid_bit == 1){
                    // Increment the duration of the valid instruction in issue queue
                    issue_queue[i].instr_in_iq.issue_queue.duration++;
                    if(issue_queue[i].instr_in_iq.issue_queue.begin_cycle == 0){
                        issue_queue[i].instr_in_iq.issue_queue.begin_cycle = cycle_count; // Change the begin cycle count for issue stage
                    }
                }
            }
            // Find the Width Oldest Valid Instructions
            int width_counter = 0;
            for(int i=0; i < iq_size; i++){
                // Check the valid and ready instructions in Issue Queue (Already sorted by sequence Id. Smallest seq id indicates oldest instr
                if((issue_queue[i].valid_bit == 1) && (issue_queue[i].rs1_rdy == 1) && (issue_queue[i].rs2_rdy == 1)){
                    execute_list.push_back(issue_queue[i].instr_in_iq); // Push the instruction to next stage - execute list
                    RemoveIssueFromIQ(issue_queue[i].sequence_id); // Remove the issue from IQ making space for new instructions.
                    width_counter++;
                }
                // Break the loop is already execute WIDTH instructions
                if(width_counter >= pipe_width){
                    break;
                }
            }
        }

        if(DEBUG_PRINT == true){
            printf("ISSUE. ISSUE QUEUE\n");
            displayIQ();
            printf("\n");
            printf("execute list\n");
            for (auto itr = execute_list.begin(); itr != execute_list.end(); itr++){
                display_instruction(*itr);
            }
        }

    }

    void Execute(){
        // iterate through the execute list and increment the duration
        for (auto itr = execute_list.begin(); itr != execute_list.end(); itr++){
            itr->execute.duration++;
            if(itr->execute.begin_cycle == 0){
                itr->execute.begin_cycle = cycle_count;
            }
        }

        // check for completed instructions
        for (auto itr = execute_list.begin(); itr != execute_list.end();){
            int delay = 0;
            // Get the delay based on op_type
            if(itr->op_type == 0){
                delay = 1;
            }
            else if(itr->op_type == 1){
                delay = 2;
            }
            else{
                delay = 5;
            }
            // check if the duration has passed and remove instr if cycle has passed.
            if((delay - itr->execute.duration) == 0){
                instruction_format result_itr =  *itr;
                itr = execute_list.erase(itr); // Remove from the execute list

                // itr->execute.begin_cycle = cycle_count; // not the right place
                Writeback_Register.push_back(result_itr); // Push to WriteBack Register (NO Limit of size)

                // Set source operand ready flags in IQ, DI and RR
                // setting in IQ
                for(int i=0; i < iq_size; i++){
                    // Find the matching instr in Issue Queue
                    if(issue_queue[i].valid_bit == 1){
                        if(issue_queue[i].rs1_tag == result_itr.dst){
                            issue_queue[i].rs1_rdy = 1;
                        }
                        if(issue_queue[i].rs2_tag == result_itr.dst){
                            issue_queue[i].rs2_rdy = 1;
                        }
                    }
                }

                // setting DI and RR
                for(int i=0; i < pipe_width; i++){
                    // Find the matching src in DI
                    if(Dispatch_Register[i].src1 == result_itr.dst){
                        Dispatch_Register[i].src1_rdy = 1;
                    }
                    if(Dispatch_Register[i].src2 == result_itr.dst){
                        Dispatch_Register[i].src2_rdy = 1;
                    }

                    // Find the matching src in RR
                    if(RegRead_Register[i].src1 == result_itr.dst){
                        RegRead_Register[i].src1_rdy = 1;
                    }
                    if(RegRead_Register[i].src2 == result_itr.dst){
                        RegRead_Register[i].src2_rdy = 1;
                    }
                }
            } 
            else{
                itr++;
            }
        }
        if(DEBUG_PRINT == true){
            printf("EXECUTE. execute list\n");
            for (auto itr = execute_list.begin(); itr != execute_list.end(); itr++){
                display_instruction(*itr);
            }
            printf("\n");
            printf("WB Register\n");
            for (auto itr = Writeback_Register.begin(); itr != Writeback_Register.end(); itr++){
                display_instruction(*itr);
            }
            printf("ISSUE QUEUE\n");
            displayIQ();
        }
    }

    void Writeback(){
        // iterate through the Writeback_Register and increment the duration
        for (auto itr = Writeback_Register.begin(); itr != Writeback_Register.end(); itr++){
            itr->writeback.duration++;
        }

        // for all the instructions present in writeback, Set the instr as ready in ROB Table and push the instr to Retire Register
        for (auto itr = Writeback_Register.begin(); itr != Writeback_Register.end();){

            // find the instruction and set the rdy bit to one
            ReorderBufferBlock* match;
            match = reorder_buffer.findTheBlock(itr->dst);
            match->data.rdy = 1;

            // Remove from WB and put in RT
            instruction_format result_itr =  *itr;
            itr = Writeback_Register.erase(itr); // Remove from the Writeback_Register

            result_itr.writeback.begin_cycle = cycle_count;
            Retire_Register.push_back(result_itr);  // push it is Retire Register
        }
        if(DEBUG_PRINT == true){
            printf("WRITEBACK. WB Register\n");
            for (auto itr = Writeback_Register.begin(); itr != Writeback_Register.end(); itr++){
                display_instruction(*itr);
            }
            printf("\n");
            printf("RT Register\n");
            for (auto itr = Retire_Register.begin(); itr != Retire_Register.end(); itr++){
                display_instruction(*itr);
            }
            printf("Reorder Buffer\n");
            reorder_buffer.displayROB();
        }
    }

    // Retire Stage Function. Should retire based on WIDTH and ReorderBuffer. Retiring instr should update RMT
    void Retire(){
        if(DEBUG_PRINT == true){
            printf("RETIRE STAGE\n");
        }

        // iterate through the Retire Register and increment the duration
        for (auto itr = Retire_Register.begin(); itr != Retire_Register.end(); itr++){
            itr->retire.duration++;
            if(itr->retire.begin_cycle == 0){
                itr->retire.begin_cycle = cycle_count;
            }
        }

        for(int i=0; i<pipe_width; i++){
            if(reorder_buffer.head->data.rdy == 1){
                // find the mapping instruction in retire register
                // long int current_pc = reorder_buffer.head->data.pc;
                int rob_tag = reorder_buffer.head->rob_tag;
                for (auto itr = Retire_Register.begin(); itr != Retire_Register.end(); itr++){
                    if(itr->dst == rob_tag){
                        instruction_format result_itr =  *itr;
                        // change the src and dst from rob tag to arf reg.
                        if(result_itr.dst >= 1000){
                            result_itr.dst = reorder_buffer.head->data.dst; // dst changed
                        }
                        if(result_itr.src1 >= 1000){
                            result_itr.src1 = result_itr.src1_org; // src1 changed
                        }
                        if(result_itr.src2 >= 1000){
                            result_itr.src2 = result_itr.src2_org; // src1 changed
                        }
                        // print statement to print the instruction
                        display_instruction(result_itr);
                        // Remove from WB and put in RT
                        itr = Retire_Register.erase(itr); // Remove from the Writeback_Register
                        break;
                    }
                }
                reorder_buffer.modifyHead();

                // find the rob tag in RMT and reset the RMT
                for(int i = 0; i< register_count; i++){
                    if(rename_map_table[i].rob_tag == rob_tag){
                        rename_map_table[i].valid_bit = 0;
                        rename_map_table[i].rob_tag = -1;
                    }
                }
            }
            else{
                break; // wait till the head completes
            }
        }

        if(DEBUG_PRINT == true){
            printf("RETIRE. RT Register\n");
            for (auto itr = Retire_Register.begin(); itr != Retire_Register.end(); itr++){
                display_instruction(*itr);
            }
            printf("\n");
            printf("Reorder Buffer\n");
            reorder_buffer.displayROB();
        }
    }

    bool Advance_Cycle(){
    // check all the registers and queues
    execution_remaining = false;
    if((Retire_Register.size() != 0) || (Writeback_Register.size() != 0) || (execute_list.size() != 0) \
        || (iq_size != checkIQFreeSize()) \
        || (RegisterEmpty(Dispatch_Register) == false) || (RegisterEmpty(RegRead_Register) == false) \
        || (RegisterEmpty(Rename_Register) == false) || (RegisterEmpty(Decode_Register) == false)){

            if(DEBUG_PRINT == true){
                printf("Retire_Register : %d\n", (Retire_Register.size() != 0));
                printf("Writeback_Register : %d\n", (Writeback_Register.size() != 0));
                printf("execute_list : %d\n", (execute_list.size() != 0));
                printf("Issue Queue : %d\n", (iq_size != checkIQFreeSize()));
                printf("Dispatch_Register : %d\n", (RegisterEmpty(Dispatch_Register) == false));
                printf("RegRead_Register : %d\n", (RegisterEmpty(RegRead_Register) == false));
                printf("Rename_Register : %d\n", (RegisterEmpty(Rename_Register) == false));
                printf("Decode_Register : %d\n", (RegisterEmpty(Decode_Register) == false));
            }
            execution_remaining = true;
        }

    // Check trace file and registers
    if(execution_remaining == false && file_fetch_remaining == false){
        return false;
    }
    else{
        cycle_count++;
        return true;
    }
}
};








/*  argc holds the number of command line arguments
    argv[] holds the commands themselves

    Example:-
    sim 256 32 4 gcc_trace.txt
    argc = 5
    argv[0] = "sim"
    argv[1] = "256"
    argv[2] = "32"
    ... and so on
*/


int main (int argc, char* argv[])
{
    FILE *FP;               // File handler
    char *trace_file;       // Variable that holds trace file name;
    proc_params params;       // look at sim_bp.h header file for the the definition of struct proc_params
    // int op_type, dest, src1, src2;  // Variables are read from trace file
    // uint64_t pc; // Variable holds the pc read from input file
    
    if (argc != 5)
    {
        printf("Error: Wrong number of inputs:%d\n", argc-1);
        exit(EXIT_FAILURE);
    }
    
    params.rob_size     = strtoul(argv[1], NULL, 10);
    params.iq_size      = strtoul(argv[2], NULL, 10);
    params.width        = strtoul(argv[3], NULL, 10);
    trace_file          = argv[4];
    // printf("rob_size:%lu "
    //         "iq_size:%lu "
    //         "width:%lu "
    //         "tracefile:%s\n", params.rob_size, params.iq_size, params.width, trace_file);

    // Open trace_file in read mode
    FP = fopen(trace_file, "r");
    if(FP == NULL)
    {
        // Throw error and exit if fopen() failed
        printf("Error: Unable to open file %s\n", trace_file);
        exit(EXIT_FAILURE);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // The following loop just tests reading the trace and echoing it back to the screen.
    //
    // Replace this loop with the "do { } while (Advance_Cycle());" loop indicated in the Project 3 spec.
    // Note: fscanf() calls -- to obtain a fetch bundle worth of instructions from the trace -- should be
    // inside the Fetch() function.
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
    Pipeline current_pipeline(params.rob_size, params.iq_size, params.width);

    do{
        if(DEBUG_PRINT == true){
            cout << endl << "===> Current Cycle : " << cycle_count << endl;
        }

        // Retire Stage Retire();
        current_pipeline.Retire();

        // Writeback Stage Writeback();
        current_pipeline.Writeback();

        // Execute Stage Execute();
        current_pipeline.Execute();

        // Issue Stage Issue();
        current_pipeline.Issue();

        // Dispatch Stage Dispatch();
        current_pipeline.Dispatch();

        // Rename Stage RegRead();
        current_pipeline.RegRead();

        // Rename Stage Rename();
        current_pipeline.Rename();

        // Decode Stage Decode();
        current_pipeline.Decode();

        // Fetch Stage Fetch();
        current_pipeline.Fetch(FP);

    } while (current_pipeline.Advance_Cycle());
    
    // while(fscanf(FP, "%lx %d %d %d %d", &pc, &op_type, &dest, &src1, &src2) != EOF){
    //     // printf("%lx %d %d %d %d\n", pc, op_type, dest, src1, src2); //Print to check if inputs have been read correctly

    // }

    printf("# === Simulator Command =========\n");
    printf("# %s %lu %lu %lu %s\n", argv[0], params.rob_size, params.iq_size, params.width, trace_file);

    printf("# === Processor Configuration ===\n");
    printf("# ROB_SIZE = %lu\n", params.rob_size);
    printf("# IQ_SIZE  = %lu\n", params.iq_size);
    printf("# WIDTH    = %lu\n", params.width);

    printf("# === Simulation Results ========\n");
    printf("# Dynamic Instruction Count    = %u\n", seq_no);
    printf("# Cycles                       = %u\n", cycle_count+1);
    printf("# Instructions Per Cycle (IPC) = %.2f\n", (float) (seq_no+1)/(cycle_count+1));
    return 0;
}
