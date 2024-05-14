@ ARM Assembly Example
@	Write an ARM Assembly program to read a number (N) and  
@	and print numbers from 1 to N


	.text	@ instruction memory
	
	.global main
main:
	@ stack handling
	@ push (store) lr to the stack
	sub	sp, sp, #4
	str	lr, [sp, #0]


	@allocate stack for input
	sub	sp, sp, #4

	@printf for int1
	ldr	r0, =formati
	bl	printf

	@scanf for int1
	ldr	r0, =formats
	mov	r1, sp	
	bl	scanf	@scanf("%d",sp)

	@copy int1 from the stack to register r4
	ldr	r4, [sp]

	@release stack
	add	sp, sp, #4

	@counter
	mov	r5, #1

loop:
	cmp	r5, r4
	bgt	exit
	
	@print r4
	ldr	r0, =formatp
	mov	r1, r5
	bl 	printf

	add	r5, r5, #1
	b	loop
	
exit:


   	@ stack handling (pop lr from the stack) and return
	ldr	lr, [sp, #0]
	add	sp, sp, #4
	mov	pc, lr		
	
	.data	@ data memory
formati: .asciz "Enter integer: "
formats: .asciz "%d"
formatp: .asciz "%d\n"
