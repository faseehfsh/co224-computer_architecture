@ ARM Assembly Example
@	Write an ARM Assembly program to read two numbers and print 
@	whether they are equal or not	


	.text	@ instruction memory
	
	.global main
main:
	@ stack handling, 
	@ push (store) lr to the stack
	sub	sp, sp, #4
	str	lr, [sp, #0]


	@allocate stack for input
	sub	sp, sp, #4

	@printf for int1
	ldr	r0, =format1
	bl	printf

	@scanf for int1
	ldr	r0, =formats
	mov	r1, sp	
	bl	scanf	@scanf("%d",sp)

	@copy int1 from the stack to register r4
	ldr	r4, [sp]

	@printf for int2
	ldr	r0, =format2
	bl	printf

	@scanf for int2
	ldr	r0, =formats
	mov	r1, sp	
	bl	scanf	@scanf("%d",sp)
	
	@copy int2 from stack to register r5
	ldr	r5, [sp]
	
	@release stack
	add	sp, sp, #4

	cmp	r4, r5
	bne	notequal

	ldr	r0, =formate
	b	exit	

notequal:
	ldr	r0, =formatn

exit:
	bl printf


    @ stack handling (pop lr from the stack) and return
	ldr	lr, [sp, #0]
	add	sp, sp, #4
	mov	pc, lr		
	
	.data	@ data memory
format1: .asciz "Enter integer1: "
format2: .asciz "Enter integer2: "
formats: .asciz "%d"
formate: .asciz "They are equal\n"
formatn: .asciz "They are not equal\n"
