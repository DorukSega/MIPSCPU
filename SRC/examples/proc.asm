addi $a0, $0, 1
addi $a1, $0, 2
jal test
j exit
test: add $v0, $a0, $a1
jr $ra
addi $t0, $0, 5
exit:
