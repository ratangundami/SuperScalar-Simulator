#ifndef SIM_PROC_H
#define SIM_PROC_H



typedef struct stage_duration{
    long int begin_cycle = 0;
    int duration = 0;
}stage_duration;

typedef struct instruction_format{
    long int sequence_id = -1;
    long int pc = -1;
    int op_type = -1;
    int src1 = -1;
    int src1_rdy = 0; // will be set by EX/RR instr when this instr is present in DI/RR
    int src1_org = -1; // original src1 to print in the retire
    int src2 = -1;
    int src2_rdy = 0; // will be set by EX/RR instr when this instr is present in DI/RR
    int src2_org = -1; // original src1 to print in the retire
    int dst = -1;
    stage_duration fetch;
    stage_duration decode;
    stage_duration rename;
    stage_duration reg_read;
    stage_duration dispatch;
    stage_duration issue_queue;
    stage_duration execute;
    stage_duration writeback;
    stage_duration retire;
}instruction_format;

typedef struct proc_params{
    unsigned long int rob_size;
    unsigned long int iq_size;
    unsigned long int width;
}proc_params;

typedef struct reorder_buffer_struct{
    int dst;
    unsigned int rdy;
    unsigned int mis;
    unsigned long int pc;
}reorder_buffer_struct;

typedef struct rename_map_table_struct{
    unsigned int valid_bit;
    int rob_tag;
}rename_map_table_struct;

typedef struct issue_queue_struct{
    unsigned int valid_bit = 0;
    long int sequence_id = -1;
    int dst_tag = -1;
    unsigned int rs1_rdy = 0;
    int rs1_tag = 0;
    unsigned int rs2_rdy = 0;
    int rs2_tag = 0;
    instruction_format instr_in_iq;
}issue_queue_struct;

// Put additional data structures here as per your requirement

#endif
