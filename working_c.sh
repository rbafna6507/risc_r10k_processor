declare -a TESTS=(backtrack basic_malloc bfs dft fc_forward graph insertionsort matrix_mult_rec mergesort omegalul outer_product priority_queue quicksort sort_search)

function cleanup() {
    sed -i "24s/.*/\`define DEBUG 1/" verilog/sys_defs.svh
    exit 1
}

trap cleanup SIGINT
trap cleanup SIGQUIT

failed_test=0
start_n=$1
end_n=$2
sed -i "24s/.*/\/\/\`define DEBUG 1/" verilog/sys_defs.svh
for i in $(seq ${start_n:=1} ${end_n:=6}); do
    sed -i "31s/.*/\`define N $i/" verilog/sys_defs.svh
    make nuke > /dev/null
    make cpu.out > /dev/null
    for test in "${TESTS[@]}"; do
        echo -n "$test (N=$i)"
        make $test.out > /dev/null
        diff output/$test.wb correct_out/$test.wb > /dev/null 2>&1
        wb_status=$?
        diff <(grep "@@@" output/$test.out) <(grep "@@@" correct_out/$test.out) > /dev/null 2>&1
        out_status=$?

        if [ $wb_status -ne 0 ] || [ $out_status -ne 0 ]
        then
            echo -e " - \033[0;31mFailed WB: $wb_status MEM: $out_status\033[0m"
            failed_test=1
        else
            echo -e " - \033[0;32mPassed\033[0m"
        fi
    done
    echo "=========="
done
sed -i "24s/.*/\`define DEBUG 1/" verilog/sys_defs.svh

if [ $failed_test -ne 1 ];
then
    echo "All Tests Passed!"
else
    echo "Tests Did Not Pass!"
    exit 1
fi
