# Pipelined MIPS Design

Final project of the course Digital System Design (DSD)

The HW was designed by Prof. An-Yeu Wu and his TAs.

## Description

See DSD_Final_Description_MIPS.pdf for the project description

## Result

See DSD_Presentation.pdf and DSD_Report for the results analysis.

## Synthesis and Running

#### Baseline

###### Synthesis

```
read_file -format verilog CHIP.v
source CHIP_syn.sdc
```

###### Run

1. RTL

```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
```

2. Gate-Level

```
ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+hasHazard +define+SDF +access+r
```

#### Branch Prediction

###### Synthesis

```
read_file -format verilog CHIP_Br.v
source CHIP_Br_syn.sdc
```

###### Run

1. RTL

```
ncverilog Final_tb.v CHIP_Br.v slow_memory.v +define+BrPred +access+r
```

2. Gate-Level

```
ncverilog Final_tb.v CHIP_Br_syn.v slow_memory.v tsmc13.v +define+BrPred +define+SDF +access+r
```

#### L2-Cache

###### Synthesis

```
read_file -format verilog CHIP_L2_Cache.v
source CHIP_L2_syn.sdc
```

###### Run

1. RTL

```
ncverilog Final_tb.v CHIP_L2_Cache.v slow_memory.v +define+L2Cache +access+r
```

2. Gate-Level

```
ncverilog Final_tb.v CHIP_L2_Cache_syn.v slow_memory.v tsmc13.v +define+L2Cache +define+SDF +access+r
```

#### Multiplication and Division

###### Synthesis

```
read_file -format verilog CHIP.v
source CHIP_syn.sdc
```

###### Run

1. RTL

```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+Multdiv +access+r
```

2. Gate-Level

```
ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+Multdiv +define+SDF +access+r
```