 
exe=main.app/Contents/MacOS/main 

all: $(exe)

$(exe): main.mm makefile
	clang++ -std=c++20 `pkg-config --cflags --libs glm` \
		-fsanitize=address \
		-framework AppKit -framework OpenGL main.mm -o $(exe)

run: $(exe)
	./$(exe)
