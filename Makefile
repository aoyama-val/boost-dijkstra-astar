dijkstra: dijkstra.cpp
	g++ -O2 $< -o $@

astar: astar.cpp
	g++ -O2 $< -o $@

run:
	./dijkstra 91 41
