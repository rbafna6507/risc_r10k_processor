#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", type=int)
    parser.add_argument("--rob", type=int)

    args = parser.parse_args()

    os.system('sed -i "24s/.*/\/\/\`define DEBUG 1/" verilog/sys_defs.svh')
    if args.n:
        os.system(f'sed -i "31s/.*/\`define N {args.n}/" verilog/sys_defs.svh')

    if args.rob:
        os.system(f'sed -i "38s/.*/\`define ROB_SZ {args.rob}/" verilog/sys_defs.svh')

    programs = [
        "alexnet",
        "backtrack",
        "basic_malloc",
        "bfs",
        "dft",
        "fc_forward",
        "graph",
        "insertionsort",
        "matrix_mult_rec",
        "mergesort",
        "omegalul",
        "outer_product",
        "priority_queue",
        "quicksort",
        "sort_search"
    ]

    subprocess.call(["make", "nuke"], stdout=subprocess.DEVNULL)
    print("Nuked")
    subprocess.call(["make", "cpu.out"], stdout=subprocess.DEVNULL)
    print("CPU compiled")

    running_processes = {}

    for p in programs:
        print(f"Running {p}")
        running_processes[p] = subprocess.Popen(["make", f"{p}.out"], stdout=subprocess.DEVNULL)

    for p in programs:
        running_processes[p].communicate()
        print(f"{p} Stats")
        print(open(f"output/{p}.cpi").read())

        print()


if __name__ == "__main__":
    main()