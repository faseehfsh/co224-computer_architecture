@Group no 19
@ E/19/105 FAHMAN M.H.M,  E/19/106 FASEEH M.F.M
@ARM assembly to reverse a specified number of input strings

	.text     @Instruction memory
	.global  main

main:	@stack handling
	sub sp,sp,#4
	str lr,[sp,#0]


	@Allocate stack for input
	sub sp,sp,#4

	@Print the prompt message
	ldr r0,=formatp1
	bl printf

	ldr r0,=formats1    @scanf for the number of strings
	mov r1,sp
	bl scanf

	ldr r1,[sp,#0]      @save the number of strings in r1
	add sp,sp,#4

	cmp r1,#0       @check for negative number
	blt invalid


	mov r4,r1
@take input strings in loop
loop1:	cmp r1,#0
	beq exit        @if number is 0 exit


	sub sp,sp,#200  @else


	mov r5,r1
	sub r1,r4,r1

	ldr r0,=formatp2    @prompt message "Enter the input string%d"
	bl printf

	ldr r0,=formatp6
	bl scanf

	ldr r0,=formats2    @take string input
	mov r1,sp
	bl scanf

	ldr r0,=formatp3    @prompt message "Output string %d is..."
    mov r1,r5
    sub r1,r4,r1
	bl printf

    mov r0,sp
	bl printReverse     @Call the function to print the string reversely

    add sp,sp,#200      @clear stack

	ldr r0,=formatp6    @newline
	bl printf

	mov r1,r5
	sub r1,r1,#1
	b loop1


invalid:ldr r0,=formatp4    @if invalid
	bl printf
	b exit

printReverse:   sub sp,sp,#20   @
    str lr,[sp,#16]
    str r6,[sp,#12]
    str r5,[sp,#8]
    str r7,[sp,#4]
    str r4,[sp,#0]


    mov r7,r0
    mov r5,#0

loop2:  add r4,r7,r5    @loop until find the null character
    ldrb r6,[r4,#0]

    cmp r6,#0
    beq loop3

    add r5,r5,#1
    b loop2

loop3:  cmp r5,#0       @loop reversely and print character by character
    beq end

    sub r4,r4,#1
    ldrb r1,[r4,#0]

    ldr r0,=formatp5
    bl printf

    sub r5,r5,#1
    b loop3

end:    ldr lr,[sp,#16] @release stack and move out from the function
    ldr r6,[sp,#12]
    ldr r5,[sp,#8]
    ldr r7,[sp,#4]
    ldr r4,[sp,#0]
    add sp,sp,#20
    mov pc,lr

exit:	ldr lr,[sp,#0]  @exit
	add sp,sp,#4
	mov pc,lr

	.data
formatp1: .asciz "Enter the number of strings :\n"
formatp2: .asciz "Enter the input string %d:\n"
formatp3: .asciz "Output string %d is...\n"
formatp4: .asciz "Invalid Number\n"
formatp5: .asciz "%c"
formatp6: .asciz "\n"
formats1: .asciz "%d"
formats2: .asciz "%[^\n]"


