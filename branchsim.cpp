#include "branchsim.hpp"

static Counter_t *gselect_counter_table = nullptr;
static uint64_t gselect_ghr = 0;
static uint64_t gselect_p = 0;
static uint64_t gselect_h = 0;

static Counter_t *gsplit_counter_table = nullptr;
static uint64_t gsplit_ghr = 0;
static uint64_t gsplit_p = 0;
static uint64_t gsplit_h = 0;

void gselect_init_predictor(branchsim_conf_t *sim_conf) {
    // create Counter table of 2^(P+H) counters and init to weakly not taken
    // 保存配置参数
    gselect_p = sim_conf->P;
    gselect_h = sim_conf->H;

    // 计算计数器表大小 2^(P+H)
    uint64_t table_size = 1ULL << (gselect_p + gselect_h);
    
    // 分配内存并初始化计数器
    gselect_counter_table = (Counter_t *)malloc(table_size * sizeof(Counter_t));
    for (uint64_t i = 0; i < table_size; i++) {
        Counter_init(&gselect_counter_table[i], 2); // 2-bit 计数器
    }

    // 初始化GHR为0
    gselect_ghr = 0;
#ifdef DEBUG
    printf("GSelect: Creating a Counter table of %" PRIu64 " 2-bit saturating counters\n", table_size); // TODO: FIX ME
#endif
}

bool gselect_predict(branch_t *br) {
#ifdef DEBUG
    printf("\tGSelect: Predicting... \n"); // PROVIDED
#endif

    // add your code here
    // 提取PC[2+P-1:2]部分（共P位）
    uint64_t pc_bits = (br->ip >> 2) & ((1ULL << gselect_p) - 1);
    
    // 提取GHR[H-1:0]部分（共H位）
    uint64_t ghr_bits = gselect_ghr & ((1ULL << gselect_h) - 1);
    
    // 组合索引 = (pc_bits << H) | ghr_bits
    uint64_t index = (pc_bits << gselect_h) | ghr_bits;
    index %= (1ULL << (gselect_p + gselect_h)); // 确保索引不越界
    
    // 获取计数器状态
    bool prediction = Counter_isTaken(&gselect_counter_table[index]);
#ifdef DEBUG
    printf("\t\tHistory: 0x%" PRIx64 ", Counter index: 0x%" PRIx64 ", Prediction: %d\n", 
        gselect_ghr, index, prediction); // TODO: FIX ME
#endif
    return prediction; // TODO: FIX ME
}

void gselect_update_predictor(branch_t *br) {
#ifdef DEBUG
    printf("\tGSelect: Updating based on actual behavior: %d\n", (int) br->is_taken); // PROVIDED
#endif

    // add your code here
    // 计算索引（与预测时相同逻辑）
    uint64_t pc_bits = (br->ip >> 2) & ((1ULL << gselect_p) - 1);
    uint64_t ghr_bits = gselect_ghr & ((1ULL << gselect_h) - 1);
    uint64_t index = (pc_bits << gselect_h) | ghr_bits;
    index %= (1ULL << (gselect_p + gselect_h));

    // 更新计数器
    Counter_update(&gselect_counter_table[index], br->is_taken);

    uint64_t old_geselect_ghr = gselect_ghr; 
    // 更新GHR：左移1位，最低位设为实际结果
    gselect_ghr = (gselect_ghr << 1) | (br->is_taken ? 1 : 0);
    gselect_ghr &= (1ULL << gselect_h) - 1; // 保留H位

#ifdef DEBUG
    printf("\t\tHistory: 0x%" PRIx64 ", Counter index: 0x%" PRIx64 ", New Counter value: 0x%" PRIx64 ", New History: 0x%" PRIx64 "\n", 
        old_geselect_ghr, index, Counter_get(&gselect_counter_table[index]), gselect_ghr); // TODO: FIX ME
#endif
}

void gselect_cleanup_predictor() {
    // add your code here
    free(gselect_counter_table);
    gselect_counter_table = nullptr;
}

void gsplit_init_predictor(branchsim_conf_t *sim_conf) {
    // create Counter table of 2^(2*H-P) counters and init to weakly not taken
    gsplit_p = sim_conf->P;
    gsplit_h = sim_conf->H;

    uint64_t table_size = 1ULL << (2 * gsplit_h - gsplit_p);
    gsplit_counter_table = (Counter_t *)malloc(table_size * sizeof(Counter_t));
    for (uint64_t i = 0; i < table_size; i++) {
        Counter_init(&gsplit_counter_table[i], 2); // 2-bit counter
    }

    // initialize GHR to 0
    gsplit_ghr = 0;

#ifdef DEBUG
    printf("GSplit: Creating a Counter table of %" PRIu64 " 2-bit saturating counters\n", table_size); // TODO: FIX ME
#endif
}

bool gsplit_predict(branch_t *br) {
#ifdef DEBUG
    printf("\tGSplit: Predicting... \n"); // PROVIDED
#endif

    // add your code here
    uint64_t pc = (br->ip >> 2) & ((1ULL << gsplit_h) - 1);
    uint64_t pc_hash = pc & ((1ULL << (gsplit_h - gsplit_p)) - 1);
    uint64_t hist_hash = gsplit_ghr & ((1ULL << (gsplit_h - gsplit_p)) - 1);
    uint64_t xor_pc_ghr = pc ^ gsplit_ghr;
    uint64_t share_hash = (xor_pc_ghr >> (gsplit_h - gsplit_p)) & ((1ULL << gsplit_p) - 1);
    uint64_t index = (share_hash << (2 * (gsplit_h - gsplit_p))) | (hist_hash << (gsplit_h - gsplit_p)) | pc_hash;
    index %= (1ULL << (2 * gsplit_h - gsplit_p));

    bool prediction = Counter_isTaken(&gsplit_counter_table[index]);

#ifdef DEBUG
    printf("\t\tHistory: 0x%" PRIx64 ", Counter index: 0x%" PRIx64 ", Prediction: %d\n", 
        gsplit_ghr, index, prediction); // TODO: FIX ME
#endif
    return prediction; // TODO: FIX ME
}

void gsplit_update_predictor(branch_t *br) {
#ifdef DEBUG
    printf("\tGSplit: Updating based on actual behavior: %d\n", (int) br->is_taken); // PROVIDED
#endif

    // add your code here
    uint64_t pc = (br->ip >> 2) & ((1ULL << gsplit_h) - 1);
    uint64_t pc_hash = pc & ((1ULL << (gsplit_h - gsplit_p)) - 1);
    uint64_t hist_hash = gsplit_ghr & ((1ULL << (gsplit_h - gsplit_p)) - 1);
    uint64_t xor_pc_ghr = pc ^ gsplit_ghr;
    uint64_t share_hash = (xor_pc_ghr >> (gsplit_h - gsplit_p)) & ((1ULL << gsplit_p) - 1);
    uint64_t index = (share_hash << (2 * (gsplit_h - gsplit_p))) | (hist_hash << (gsplit_h - gsplit_p)) | pc_hash;
    index %= (1ULL << (2 * gsplit_h - gsplit_p));

    Counter_update(&gsplit_counter_table[index], br->is_taken);

    uint64_t old_gsplit_ghr = gsplit_ghr;
    gsplit_ghr = (gsplit_ghr << 1) | (br->is_taken ? 1 : 0);
    gsplit_ghr &= (1ULL << gsplit_h) - 1; 

#ifdef DEBUG
    printf("\t\tHistory: 0x%" PRIx64 ", Counter index: 0x%" PRIx64 ", New Counter value: 0x%" PRIx64 ", New History: 0x%" PRIx64 "\n",
        old_gsplit_ghr, index, Counter_get(&gsplit_counter_table[index]), gsplit_ghr); // TODO: FIX ME
#endif
}

void gsplit_cleanup_predictor() {
    // add your code here
    free(gsplit_counter_table);
    gsplit_counter_table = nullptr;
}

/**
 *  Function to update the branchsim stats per prediction
 *
 *  @param prediction: The prediction returned from the predictor's predict function
 *  @param br: Pointer to the branch_t that is being predicted -- contains actual behavior
 *  @param sim_stats Pointer: to the simulation statistics -- update in this function
*/
void branchsim_update_stats(bool prediction, branch_t *br, branchsim_stats_t *sim_stats) {
    // add your code here
    // 分支指令数加1
    sim_stats->num_branch_instructions++;
    
    // 比较预测结果与实际结果
    if (prediction == br->is_taken) {
        sim_stats->num_branches_correctly_predicted++;
    } else {
        sim_stats->num_branches_mispredicted++;
    }
}

/**
 *  Function to cleanup branchsim statistic computations such as prediction rate, etc.
 *
 *  @param sim_stats Pointer to the simulation statistics -- update in this function
*/
void branchsim_finish_stats(branchsim_stats_t *sim_stats) {
    // add your code here
    // 计算分支指令比例
    if (sim_stats->total_instructions > 0) {
        sim_stats->fraction_branch_instructions = 
            (double)sim_stats->num_branch_instructions / sim_stats->total_instructions;
    } else {
        sim_stats->fraction_branch_instructions = 0.0;
    }
    
    // 计算预测准确率
    if (sim_stats->num_branch_instructions > 0) {
        sim_stats->prediction_accuracy = 
            (double)sim_stats->num_branches_correctly_predicted / sim_stats->num_branch_instructions;
    } else {
        sim_stats->prediction_accuracy = 0.0;
    }
}


