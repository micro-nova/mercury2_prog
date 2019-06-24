mercury2_prog: mercury2_prog.c
	gcc -O3 -o mercury2_prog mercury2_prog.c d2xx_linux/libftd2xx.a -pthread -ldl -Id2xx_linux
	x86_64-w64-mingw32-gcc-7.3-win32 -O3 -o mercury2_prog.exe mercury2_prog.c d2xx_win/ftd2xx.lib -Id2xx_win
clean: 
	rm -f mercury2_prog mercury2_prog.exe
