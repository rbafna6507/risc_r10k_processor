#!/bin/bash

# Define the C tests array based on the files shown with "C" icon
# declare -a C_TESTS=(alexnet backtrack basic_malloc bfs dft fc_forward graph insertionsort matrix_mult_rec mergesort omegalul outer_product priority_queue quicksort sort_search)
declare -a C_TESTS=(backtrack basic_malloc bfs dft graph insertionsort matrix_mult_rec mergesort omegalul priority_queue quicksort sort_search)

# Define optimization flags array
declare -a OPT_FLAGS=("O0" "O" "O2" "O3" "Os")

failed_test=0
# Only run optimization tests if -c flag was specified
echo "Testing C files with different optimization flags..."
echo "=========="

for opt_flag in "${OPT_FLAGS[@]}"; do
    echo "Testing with -$opt_flag"
    # Replace the optimization flag in the Makefile
    sed -i "127s/OFLAGS     = .*/OFLAGS     = -$opt_flag/" Makefile

    for i in $(seq 1 6); do
        echo "N=$i with -$opt_flag"
        sed -i "31s/.*/\`define N $i/" verilog/sys_defs.svh
        make nuke > /dev/null
        make cpu.out > /dev/null

        for test in "${C_TESTS[@]}"; do
            echo -n "$test (N=$i, -$opt_flag)"
            make $test.out > /dev/null
            diff output/$test.wb correct_out/$opt_flag/$test.wb > /dev/null 2>&1
            wb_status=$?
            diff <(grep "@@@" output/$test.out) <(grep "@@@" correct_out/$opt_flag/$test.out) > /dev/null 2>&1
            out_status=$?

            if [ $wb_status -ne 0 ] || [ $out_status -ne 0 ]
            then
                echo -e " - \033[0;31mFailed WB: $wb_status MEM: $out_status\033[0m"
                failed_test=1
            else
                echo -e " - \033[0;32mPassed\033[0m"
            fi
            echo ""
        done
        echo "----------"
    done
    echo "=========="
done

if [ $failed_test -ne 1 ];
then
    echo "All Tests Passed!"
else
    echo "Tests Did Not Pass!"
    exit 1
fi
