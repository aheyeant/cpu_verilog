.data                  
.align  2              # set data alignment to 4 bytes
.text                  # beginning of the text segment (or code segment)

start:
lw $s2, 8($0)
beq $s2, $0, done
addi $s2, $s2, -1
addi $s0, $0, 12

big_for:
beq $s2, $0 done	
add $t0, $0, $s2
addi $s2, $s2, -1
add $s1, $0, $s0


nextStep:
beq $t0, $0, big_for
lw $t1, 0($s1)
lw $t2, 4($s1)
slt $t3, $t1, $t2
beq $t3, $0, swap


continue:
addi $s1, $s1, 4
addi $t0, $t0, -1
j nextStep


swap:
sw $t1, 4($s1)
sw $t2, 0($s1) 
j continue

done:

