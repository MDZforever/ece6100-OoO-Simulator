#!/bin/bash
set -e

source validate_common.sh
source validate_undergrad_p1.sh
source validate_undergrad_p2.sh

main() {
    mkdir -p "$student_stat_dir"

    main_undergrad_p1
    main_undergrad_p2
}

main