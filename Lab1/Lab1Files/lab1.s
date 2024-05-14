@ ARM Assembly - Lab 1
@ E Number :
@ Name :

	.text 	@ instruction memory
	.global main
main:
	@ stack handling, will discuss later
	@ push (store) lr to the stack
	sub r13, r13, #4
	str lr, [r13, #0]

	@ load values
	ldr r0, =array_a
	ldr r1, =array_b
	mov r2, #3
	mov r3, #7
	mov r4, #10

	
	@ Write YOUR CODE HERE
	@ b[4] = 6 + a[9] - a[3] + b[1] – ( c + d - e )
	@ Base address of a in r0
	@ Base address of b in r1
	@ c,d,e in r2,r3,r4 respectively 

	@ ---------------------
	
 	LDR r5,[r0,#36]
 	LDR r6,[r0,#12]
 	LDR r7,[r1,#4]
 	
 	ADD r8,r5,#6
 	ADD r8,r8,r7
 	SUB r8,r8,r6
 	
 	ADD r9,r2,r3
 	SUB r9,r9,r4
	SUB r10,r8,r9
	
	STR r10,[r1,#16]

	@ ---------------------
	
	
	@ load aguments and print
	ldr r0, =format
	ldr r2, =array_b
	ldr r1, [r2,#16]
	bl printf

	@ stack handling (pop lr from the stack) and return
	ldr lr, [r13, #0]
	add r13, r13, #4
	mov pc, lr

	
	
	.data	@ data memory
	
format: .asciz "The Answer is %d (Expect -3 if correct)\n"
	
	@array called array_a of size 40 bytes
	.balign 4 			@align to an address of multiple of 4
array_a: .word 1,5,7,67,5,54,65,23,34,54

	@array called array_b of size 20 bytes
	.balign 4 			@align to an address of multiple of 4
array_b: .word 7,4,8,3,5
