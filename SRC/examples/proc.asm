addi $a0, $0, 1
addi $t0, $0, 5
jal test
j exit
test: add $v0, $a0, $v0
slt $t1, $t0, $v0
beq $t1, $0, test
jr $ra
exit:
