 	.text     @Instruction memory
        .global  main

main:   @stack handling
        sub sp,sp,#4
        str lr,[sp,#0]

	sub sp,sp,#100
	
	ldr r0,=formats
	mov r1,sp
	bl scanf

	ldrb r3,[sp],#0

	ldr r0,=formatp
	mov r1,r3
	bl printf

	add sp,sp,#100
	ldr lr,[sp,#0]
	add sp,sp,#4
	mov pc,lr

	.data
formats: .asciz "%s"
formatp: .asciz "%s\n"

