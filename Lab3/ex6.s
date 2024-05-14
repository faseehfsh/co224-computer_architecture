@ ARM Assembly - exercise 6 
@ Group Number : 19

	.text 	@ instruction memory
	
	
@ Write YOUR CODE HERE	

@ ---------------------

@factorial value in r3
@ counting variable (i) in r5
	
fact:	
	mov r3,#1
	mov v2,#1

loop:	cmp v2,r0
	BGT exit	

	mul r6,r3,v2
	mov r3,r6
	add v2,v2,#1	
	B loop

exit:	mov r0,r3
	mov pc,lr





@ ---------------------	

.global main
main:
	@ stack handling, will discuss later
	@ push (store) lr to the stack
	sub sp, sp, #4
	str lr, [sp, #0]

	mov r4, #8 	@the value n

	@ calling the fact function
	mov r0, r4 	@the arg1 load
	bl fact
	mov r5,r0
	

	@ load aguments and print
	ldr r0, =format
	mov r1, r4
	mov r2, r5
	bl printf

	@ stack handling (pop lr from the stack) and return
	ldr lr, [sp, #0]
	add sp, sp, #4
	mov pc, lr

	.data	@ data memory
format: .asciz "Factorial of %d is %d\n"

