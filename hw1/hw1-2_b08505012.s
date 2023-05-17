.globl __start

.rodata
    msg0: .string "This is HW1-2: \n"
    msg1: .string "Plaintext:  "
    msg2: .string "Ciphertext: "
.text

################################################################################
  # print_char function
  # Usage: 
  #     1. Store the beginning address in x20
  #     2. Use "j print_char"
  #     The function will print the string stored from x20 
  #     When finish, the whole program with return value 0

print_char:
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    add a1,x0,x20
    ecall

  # Ends the program with status code 0
    addi a0,x0,10
    ecall
    
################################################################################

__start:
  # Prints msg
    addi a0, x0, 4
    la a1, msg0
    ecall

    la a1, msg1
    ecall
    
    addi a0,x0,8
    li a1, 0x10130
    addi a2,x0,2047
    ecall
    
  # Load address of the input string into a0
    add a0,x0,a1
    

################################################################################ 
  # Write your main function here. 
  # a0 stores the begining Plaintext
  # Do store 66048(0x10200) into x20 
  # ex. j print_char
init:
    li x20, 0x10200        
    add t3, x0, x20        # point to x20 and increase in each loop
    li t1, 32              # t1 check if there is a spaces
    li t2, 0               # t2 represent the occurence of the space
    
caesar:
    lb t0, 0(a0)           # load Byte plaintext from a0
    beq t0, zero, end      # if t0 == 0, end of the text, jump to end
    beq t0, t1, space      # if t0 == 32, jump to space function
    
    addi t0, t0, 3         # plus 3 to implement caesar encryption
    sb t0, 0(t3)           # store the ciphertext to x20 array
    addi a0, a0, 1         # shift the pointer
    addi t3, t3, 1         # shift the pointer
    
    jal caesar
    
space:
    addi t0, t2, 48        # in ascii code, 48 is the place representing numbers
    addi t2, t2, 1         # space occurence plus one
    
    sb t0, 0(t3)           # store the ciphertext
    addi a0, a0, 1         # shift the pointer
    addi t3, t3, 1         # shift the pointer
    
    jal caesar
    
end:
    j print_char
  
################################################################################

