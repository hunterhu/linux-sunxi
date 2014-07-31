#!/bin/bash

# -o : boolean OR operator

#replace it with your linux kernel directory
LNX=$HOME/projects/1030tpc-dev/sunxi-bsp/linux-sunxi
cd / 	
find  $LNX \
	    -path "$LNX/arch/*" ! -path "$LNX/arch/arm*" -prune -o \
        -path "$LNX/arch/arm/*" ! -path "$LNX/arch/arm/mach-msm*" -prune -o \
		-path "$LNX/include/asm-*" ! -path "$LNX/include/asm-arm*" -prune -o \
	    -path "$LNX/tmp*" -prune -o \
		-name "*.[chxsS]" -print >$LNX/cscope.files

#note: 
#      According to "man find" notes that "the pattern match test applies to 
#      the whole file name,so need the * at the end".
#      That means the following line won't work:
#          -path "$LNX/arch" ! -path "$LNX/arch/arm" -prune -o \
#      Have to do this with a *:
#          -path "$LNX/arch/*" ! -path "$LNX/arch/arm/*" -prune -o \

