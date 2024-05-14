@ ARM Assembly - lab 3.2 
@ Group Number :

	.text 	@ instruction memory

	
@ Write YOUR CODE HERE	

@ ---------------------	
gcd:	sub sp,sp,#8
	str lr,[sp,#4]
	str r4,[sp,#0]

	cmp r1,#0
	beq direct
	
	mov r4,r1
	b loop

else:	sub r4,r4,r0
	sub r4,r1,r4
	mov r0,r1
	mov r1,r4

	bl gcd
	ldr lr,[sp,#4]
	ldr r4,[sp,#0]
	add sp,sp,#8
	mov pc,lr

loop:	cmp r4,r0
	bgt else

	add r4,r4,r1
	b loop

direct:	add sp,sp,#8
	mov pc,lr
	
@ ---------------------	

	.global main
main:
	@ stack handling, will discuss later
	@ push (store) lr to the stack
	sub sp, sp, #4
	str lr, [sp, #0]

	mov r4, #64 	@the value a
	mov r5, #24 	@the value b
	

	@ calling the mypow function
	mov r0, r4 	@the arg1 load
	mov r1, r5 	@the arg2 load
	bl gcd
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
format: .asciz "gcd(%d,%d) = %d\n"

