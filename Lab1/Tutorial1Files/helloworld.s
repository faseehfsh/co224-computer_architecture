@ ARM Assembly - Hello World! 
@ 
@ Roshan Ragel - roshanr@pdn.ac.lk
@ Hasindu Gamaarachchi - hasindu@ce.pdn.ac.lk
@
@ Assemble as : arm-linux-gnueabi-gcc -Wall example.s -o helloworld
@ Run as : qemu-arm -L /usr/arm-linux-gnueabi helloworld



	.text 	@ instruction memory starts here
	.global main @ makes the main function visible for the linker (will talk later)
	@ things starting with '.' are called assembler directives
	@ instructions that directs the assembler (program that convers instructions to machine code) to do something
	
	
@main program starts here	
main:
	
	@ stack handling, will discuss later
	sub sp, sp, #4
	str lr, [sp, #0]

	@ load arguments and call printf
	ldr r0, =hello
	bl printf

	@ stack handling and return
	ldr lr, [sp, #0]
	add sp, sp, #4
	mov pc, lr

	
	.data	@ data memory starts here

hello:	.asciz "Hello World\n"
