test.out: ./tests/main.cpp ./src/*.h
	g++ -std=c++14 tests/main.cpp -o test.out -L/usr/local/lib -lgtest -lgtest_main -lpthread

.PHONY: test
test: test.out
	./test.out