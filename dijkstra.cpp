#include <iostream>
#include <vector>
#include <deque>
#include <string>
#include <set>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace boost;

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, boost::no_property, boost::property<boost::edge_weight_t, int> > Graph;
typedef std::pair<int, int>                             Edge;
typedef boost::graph_traits<Graph>::vertex_descriptor   Vertex;

struct timespec tsStart;
struct timespec tsEnd;

// グラフを作る
Graph make_graph(const char* filename)
{
    std::set<int> vertices;
    std::vector<Edge> edges;
    std::vector<int> weights;
    char buf[256];
    FILE* fp = fopen(filename, "r");

    if (!fp) {
        fprintf(stderr, "Cound not open: %s\n", filename);
        exit(1);
    }

    while (!feof(fp)) {
        char type[10];
        int from;
        int to;
        int weight;
        fgets(buf, sizeof(buf), fp);
        if (buf[0] == 'a') {
            sscanf(buf, "%s %d %d %d", type, &from, &to, &weight);
            vertices.insert(from);
            vertices.insert(to);
            edges.push_back(std::make_pair(from, to));
            weights.push_back(weight);
        }
    }

    fclose(fp);
    return Graph(edges.begin(), edges.end(), weights.begin(), vertices.size());
}

void startTimer()
{
    clock_gettime(CLOCK_REALTIME, &tsStart);
}

void endTimer()
{
    clock_gettime(CLOCK_REALTIME, &tsEnd);
}

void printTimer()
{
    printf("Time: %g sec\n", (double)(tsEnd.tv_sec - tsStart.tv_sec) + 1.0e-9 * (tsEnd.tv_nsec - tsStart.tv_nsec));
}

void dijkstra(const Graph& g, Vertex from, Vertex to)
{
    startTimer();

    // 最短経路を計算
    std::vector<Vertex> parents(boost::num_vertices(g));
    boost::dijkstra_shortest_paths(g, from, boost::predecessor_map(&parents[0]));

    endTimer();

    // 経路なし
    if (parents[to] == to) {
        std::cout << "no path" << std::endl;
        return;
    }

    // 最短経路の頂点リストを作成
    std::deque<Vertex> route;
    for (Vertex v = to; v != from; v = parents[v]) {
        route.push_front(v);
    }
    route.push_front(from);

    // 最短経路を出力
    int i = 1;
    for (const Vertex v : route) {
        std::cout << i << ": " <<  v << std::endl;
        i++;
    }

    printTimer();
}

void astar()
{
}

int main(int argc, char* argv[])
{
    if (argc < 4) {
        printf("Usage: %s GR_FILE FROM TO\n", argv[0]);
        exit(1);
    }
    const Graph g = make_graph(argv[1]);
    const Vertex from = atoi(argv[2]); // 開始地点
    const Vertex to = atoi(argv[3]); // 目的地

    dijkstra(g, from, to);

    astar();

    return 0;
}
