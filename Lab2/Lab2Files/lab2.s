@ ARM Assembly - lab 2
@ Group Number :

	.text 	@ instruction memory
	.global main
main:
	@ stack handling, will discuss later
	@ push (store) lr to the stack
	sub sp, sp, #4
	str lr, [sp, #0]

	@ load values
	
	@ Write YOUR CODE HERE
	
	@	Sum = 0;
	@	for (i=0;i<10;i++){
	@			for(j=5;j<15;j++){
	@				if(i+j<10) sum+=i*2
	@				else sum+=(i&j);	
	@			}	
	@	} 
	@ Put final sum to r5


	@ ---------------------

	@ r0=Sum, r1=i, r2=j, r3=i+j, r4=i*2, r6=i&j
	
	MOV r0,#0
	MOV r1,#-1
	
	Loop1:	CMP r1,#9
		BGE Exit
		
		MOV r2,#5
                ADD r1,r1,#1
		B Loop2

	Loop2:
		CMP r2,#15
		BGE Loop1

		ADD r3,r1,r2
		CMP r3,#10
		BGE Else

		ADD r0,r0,r1, LSL#1
		ADD r2,r2,#1		
		B Loop2
		
	Else:	AND r6,r1,r2
		ADD r0,r0,r6
		ADD r2,r2,#1
		B Loop2
		
	Exit:
	
	MOV r5,r0
	
	@ ---------------------
	
	
	@ load aguments and print
	ldr r0, =format
	mov r1, r5
	bl printf

	@ stack handling (pop lr from the stack) and return
	ldr lr, [sp, #0]
	add sp, sp, #4
	mov pc, lr

	.data	@ data memory
format: .asciz "The Answer is %d (Expect 300 if correct)\n"

