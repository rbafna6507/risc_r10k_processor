    addi x6, x6, 3
    addi x5, x5, 2
    bne x5, x6, bt1
    mul	x6,x5,x5 
bt1:
    mul	x5,x6,x6 
    addi x1, x1, 2
    wfi