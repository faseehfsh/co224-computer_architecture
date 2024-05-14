@ ARM Assembly - exercise 7 
@ Group Number :

	.text 	@ instruction memory

	
@ Write YOUR CODE HERE	

@ ---------------------	
Fibonacci:
	sub sp,sp,#12
	str lr,[sp,#0]
	str r0,[sp,#4]
	str r8,[sp,#8]

	cmp r0,#2
	ble default

	sub r0,r0,#1
	bl Fibonacci
	mov r8,r0

	ldr r0,[sp,#4]
	sub r0,r0,#2
	bl Fibonacci
	add r0,r0,r8

	ldr lr,[sp,#0]
	ldr r8,[sp,#8]
	add sp,sp,#12
	mov pc,lr


default:
	mov r0,#1
	add sp,sp,#12
	mov pc,lr
		


@ ---------------------
	
	.global main
main:
	@ stack handling, will discuss later
	@ push (store) lr to the stack
	sub sp, sp, #4
	str lr, [sp, #0]

	mov r4, #8 	@the value n

	@ calling the Fibonacci function
	mov r0, r4 	@the arg1 load
	bl Fibonacci
	mov r5,r0
	

	@ load aguments and print
	ldr r0, =format
	mov r1, r4
	mov r2, r5
	bl printf

	@ stack handling (pop lr from the stack) and return
	ldr lr,  [sp, #0]
	add sp, sp, #4
	mov pc, lr

	.data	@ data memory
format: .asciz "F_%d is %d\n"

