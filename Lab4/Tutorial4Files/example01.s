@ ARM Assembly example for using scanf("%d") and printf("%d")



	.text	@ instruction memory
	
	.global main
main:
	@ stack handling
	@ push (store) lr to the stack
	sub	sp, sp, #4
	str	lr, [sp, #0]


	@allocate stack for input
	sub	sp, sp, #4

	@scanf to get an integer
	ldr	r0, =formats
	mov	r1, sp	
	bl	scanf	@scanf("%d",sp)

	@copy from stack to register
	ldr	r1, [sp,#0]

	@release stack
	add	sp, sp, #4

	@format for printf
	ldr	r0, =formatp
	bl 	printf


    @ stack handling (pop lr from the stack) and return
	ldr	lr, [sp, #0]
	add	sp, sp, #4
	mov	pc, lr		
	
	
	.data	@ data memory
formats: .asciz "%d"
formatp: .asciz "The number is %d\n"
