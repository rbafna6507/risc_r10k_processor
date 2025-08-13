    li	x30, 0
B:  nop
    beq x30, x0, E
    addi x1, x0, 1
E:  nop
    wfi
