# modify # of the pattern here
nb_notBr   = 10
nb_interBr = 20
nb_Br      = 30
total      = nb_notBr+nb_interBr+nb_Br

assert(nb_notBr>0)
assert(nb_interBr>0)
assert(nb_Br>0)

I_mem_BrPred_file = open("I_mem_BrPred","w")
with open("I_mem_BrPred_ref", "r") as f:
    for line in f:
        if "//0x04//" in line:
            line = line.replace("a = 2"           , "a = {:d}".format(nb_notBr))
            line = line.replace("0000000000000010", "{:b}".format(nb_notBr).zfill(16))
            line = line.replace("0x0002", "0x"+"{:x}".format(nb_notBr).zfill(4).upper())
        if "//0x08//" in line:
            line = line.replace("b = 2"           , "b = {:d}".format(nb_interBr))
            line = line.replace("0000000000000010", "{:b}".format(nb_interBr).zfill(16))
            line = line.replace("0x0002", "0x"+"{:x}".format(nb_interBr).zfill(4).upper())
        if "//0x0C//" in line:
            line = line.replace("c = 2"           , "c = {:d}".format(nb_Br))
            line = line.replace("0000000000000010", "{:b}".format(nb_Br).zfill(16))
            line = line.replace("0x0002", "0x"+"{:x}".format(nb_Br).zfill(4).upper())
        if "//0x58//" in line:
            line = line.replace("a+b+c = 6"       , "a+b+c = {:d}".format(total))
        I_mem_BrPred_file.write(line)        
I_mem_BrPred_file.close()

TestBed_BrPred_file = open("TestBed_BrPred.v","w")
with open("TestBed_BrPred_ref.v", "r") as f:
    for line in f:
        if "`define	answer" in line:
            line = line.replace("6", "{:d}".format(total)) 
        TestBed_BrPred_file.write(line)
TestBed_BrPred_file.close()
