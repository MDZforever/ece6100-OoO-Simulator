# DO NOT RUN THIS SCRIPT ALONE


benchmarks=( deepsjeng531 leela541 mcf505 xz557 nab544 )
spotlight_benchmark=leela541 # test this benchmark with all configs

flags_gselect='-b 1 -P 9 -H 8'
flags_gselect_small='-b 1 -P 8 -H 7'

flags_bp_gselect='-x '"${flags_gselect}"
flags_bp_gselect_small='-x '"${flags_gselect_small}"

main_undergrad_p1() {
    banner "Testing GSelect branch predictor"
    for benchmark in "${benchmarks[@]}"; do
        generate_stats_and_diff bp_gselect "$benchmark"
    done
    generate_stats_and_diff bp_gselect_small "${spotlight_benchmark}"
}