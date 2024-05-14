@ ARM Assembly - lab 3.1
@ 
@ Roshan Ragel - roshanr@pdn.ac.lk
@ Hasindu Gamaarachchi - hasindu@ce.pdn.ac.lk

	.text 	@ instruction memory

	
@ Write YOUR CODE HERE	

@ ---------------------	
mypow:
	sub sp,sp,#16
	str lr,[sp,#12]
	str r0,[sp,#8]
	str r7,[sp,#4]
	str r8,[sp,#0]

	cmp r1,#0
	beq zeroPower
	
	cmp r1,#1
	beq onePower

	sub r1,r1,#1
	bl mypow
	mov r7,r0
	ldr r0,[sp,#8]
	mul r8,r0,r7
	mov r0,r8
	
	ldr r8,[sp,#0]
	ldr r7,[sp,#4]
	ldr lr,[sp,#12]
	add sp,sp,#16
	mov pc,lr

zeroPower:
	mov r0,#1
	add sp,sp,#16
	mov pc,lr

onePower:
	add sp,sp,#16
	mov pc,lr

@ ---------------------	

	.global main
main:
	@ stack handling, will discuss later
	@ push (store) lr to the stack
	sub sp, sp, #4
	str lr, [sp, #0]

	mov r4, #8 	@the value x
	mov r5, #3 	@the value n
	

	@ calling the mypow function
	mov r0, r4 	@the arg1 load
	mov r1, r5 	@the arg2 load
	bl mypow
	mov r6,r0
	

	@ load aguments and print
	ldr r0, =format
	mov r1, r4
	mov r2, r5
	mov r3, r6
	bl printf

	@ stack handling (pop lr from the stack) and return
	ldr lr, [sp, #0]
	add sp, sp, #4
	mov pc, lr

	.data	@ data memory
format: .asciz "%d^%d is %d\n"

