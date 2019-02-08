dijkstra: dijkstra.cpp
	g++ -O2 $< -o $@

astar: astar.cpp
	g++ -O2 $< -o $@

run:
	./dijkstra USA-road-d.NY.gr 91 41
