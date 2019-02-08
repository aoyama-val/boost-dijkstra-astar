dijkstra: dijkstra.cpp
	g++ -O2 -W -Wall -Wno-unused-parameter $< -o $@

run:
	./dijkstra 91 41
