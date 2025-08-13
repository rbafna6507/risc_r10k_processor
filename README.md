### Project

You'll want to checkout `/verilog/cpu.sv` - everything else is modules we built from scratch to use inside our cpu.

This project was a group effort with 6 individual contributors developing in Verilog. We worked together to create a RISC-V R10K processor with register renaming and equipped to handle data and structural hazards.

We also built in several advanced features: n-way superscalar, early branch resolution, branch prediction using GShare, instruction cache, instruction prefetching, and data cache. 

We achieved a 8.6ns clock period.