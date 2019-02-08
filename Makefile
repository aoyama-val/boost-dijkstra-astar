shortest_path: shortest_path.cpp
	g++ -O2 -W -Wall -Wno-unused-parameter $< -o $@

run:
	./shortest_path 91 41
