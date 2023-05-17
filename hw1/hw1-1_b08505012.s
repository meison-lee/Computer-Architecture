.globl __start

.rodata
    msg0: .string "This is HW1-1: T(n) = 4T(n/2) + 2n + 7, T(1) = 5\n"
    msg1: .string "Enter a number: "
    msg2: .string "The result is: "

.text


__start:
  # Prints msg0
    addi a0, x0, 4
    la a1, msg0
    ecall

  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall

  # Reads an int
    addi a0, x0, 5
    ecall

################################################################################ 
  # Write your main function here. 
  # Input n is in a0. You should store the result T(n) into t0
  # HW1-1 T(n) = 4T(n/2) + 2n + 7, T(1) = 5, round down the result of division
  # ex. addi t0, a0, 1
  
#function to call recursive function
init:
    jal recursive_func
    jal result

recursive_func:      
    addi  sp, sp, -8    # reserve stack area
    sw ra, 0(sp)        # save the return address
    sw a0, 4(sp)        # save a0 in stack
    li t0, 2            # assign t0 -> 2
    blt a0, t0, one     # if a0 < t0 call function "one"            
    srli a0, a0, 1      # divide a0 by 2  
    jal recursive_func  # recursive call the funtion
                    
    lw t1, 4(sp)        # load n into t1
    slli a1, a1, 2      # a1 * 4
    slli t1, t1, 1      # t1 * 2
    add a1, a1, t1      # a1 += t1
    addi a1, a1, 7      # a1 += 7
    add t0, x0, a1      # t0 = a1

    j done              # jump to done
one:
    li a1, 5            # T(1) = 5, store 5 to a1
done:
    lw ra, 0(sp)        # load return address from stack
    addi sp, sp, 8      # free our stack frame
    jr ra               # and return to ra address

################################################################################

result:
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall

  # Prints the result in t0
    addi a0, x0, 1
    add a1, x0, t0
    ecall
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall