@ ARM Assembly - exercise 2 template
@ 


	.text 	@ instruction memory
	.global main
main:
	@ stack handling, will discuss later
	@ push (store) lr to the stack
	sub sp, sp, #4
	str lr, [sp, #0]

	@ load values
	ldr r0, =array_a
	mov r1, #5
	mov r2, #3
	mov r3, #7
	str r2, [r0,#0]
	str r3, [r0,#4]
	
	@ Write YOUR CODE HERE
	@ a[2] = a[0] + a[1] - b
	@ base address of a in r0
	@ b in r1

	@ ---------------------

	LDR r2,[r0]
	LDR r3,[r0,#4]	
	ADD r4,r2,r3
	SUB r5,r4,r1
	STR r5,[r0,#8]
	

	@ ---------------------
	
	
	@ load aguments and print
	ldr r0, =format
	ldr r2, =array_a
	ldr r1, [r2,#8]
	bl printf

	@ stack handling (pop lr from the stack) and return
	ldr lr, [sp, #0]
	add sp, sp, #4
	mov pc, lr

	
	
	.data	@ data memory
	
format: .asciz "The Answer is %d (Expect 5 if correct)\n"
	
	@array called arary_a of size 400 bytes
	.balign 4 			@align to an address of multiple of 4
array_a: .skip 400		@unitilized array of 400 bytes

