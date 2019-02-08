#include <iostream>
#include <vector>
#include <deque>
#include <string>
#include <set>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/breadth_first_search.hpp>


const char* GR_FILENAME = "USA-road-d.NY.gr";
const char* CO_FILENAME = "USA-road-d.NY.co";

using namespace std;

struct timespec tsStart;
struct timespec tsEnd;

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


///////////////////////////////////////////////////////////////////////////////
//   Graph
///////////////////////////////////////////////////////////////////////////////

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, boost::no_property, boost::property<boost::edge_weight_t, int> > Graph;
typedef std::pair<int, int>                             Edge;
typedef boost::graph_traits<Graph>::vertex_descriptor   Vertex;
struct found_goal {}; // exception for termination

Graph loadGraph(const char* filename)
{
    std::set<int> vertices;
    std::vector<Edge> edges;
    std::vector<int> weights;
    FILE* fp = fopen(filename, "r");

    if (!fp) {
        fprintf(stderr, "Cound not open: %s\n", filename);
        exit(1);
    }

    int n = 0;
    char buf[256];
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        char type[10];
        int from;
        int to;
        int weight;
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

void printRoute(std::deque<Vertex>& route)
{
    for (const Vertex v : route) {
        printf("%6d ", v);
    }
    printf("\n");
}

///////////////////////////////////////////////////////////////////////////////
//   Dijkstra
///////////////////////////////////////////////////////////////////////////////

class dijkstra_one_goal_visitor : public boost::default_dijkstra_visitor {
public:
    explicit dijkstra_one_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class B_G>
        void examine_vertex(Vertex &u, B_G &) {
            if (u == m_goal) throw found_goal();
        }
private:
    Vertex m_goal;
};

void dijkstra(const Graph& g, Vertex start, Vertex goal)
{
    startTimer();

    std::vector<Vertex> parents(boost::num_vertices(g));
    std::vector<int> distances(boost::num_vertices(g));

    try {
        // 最短経路を計算
        boost::dijkstra_shortest_paths(
                                       g,
                                       start,
                                       boost::predecessor_map(&parents[0]).distance_map(&distances[0]).
                                       visitor(dijkstra_one_goal_visitor(goal))
                                      );
    } catch (found_goal fg) { // found a path to the goal
        endTimer();

        // 経路なし
        if (parents[goal] == goal) {
            printf("Route not found\n");
            return;
        }

        // 最短経路の頂点リストを作成
        std::deque<Vertex> route;
        for (Vertex v = goal; v != start; v = parents[v]) {
            route.push_front(v);
        }
        route.push_front(start);

        // 最短経路を出力
        printf("### Dijkstra\n");
        printRoute(route);
    }

    printTimer();
}


///////////////////////////////////////////////////////////////////////////////
//   A*
///////////////////////////////////////////////////////////////////////////////

struct location
{
    int y, x;
};

typedef Graph::edge_descriptor edge_descriptor;
typedef Graph::vertex_iterator vertex_iterator;
typedef std::pair<int, int> edge;

std::vector<location> g_locations;

void loadCo(const char* filename)
{
    FILE* fp = fopen(filename, "r");

    if (!fp) {
        fprintf(stderr, "Cound not open: %s\n", filename);
        exit(1);
    }

    location loc;
    g_locations.push_back(loc); // インデックスを合わせるためのダミー

    char buf[256];
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        char type[10];
        int n;
        int y;
        int x;
        if (buf[0] == 'v') {
            sscanf(buf, "%s %d %d %d", type, &n, &y, &x);
            location loc;
            loc.y = y;
            loc.x = x;
            g_locations.push_back(loc);
        }
    }

    fclose(fp);
}

// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(LocMap& l, Vertex goal)
        : m_location(l), m_goal(goal) {}
    CostType operator()(Vertex u)
    {
        CostType dx = m_location[m_goal].x - m_location[u].x;
        CostType dy = m_location[m_goal].y - m_location[u].y;
        return ::sqrt(dx * dx + dy * dy);
    }
private:
    LocMap m_location;
    Vertex m_goal;
};

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
        void examine_vertex(Vertex u, Graph& g) {
            if (u == m_goal)
                throw found_goal();
        }
private:
    Vertex m_goal;
};

void astar(const Graph& g, Vertex start, Vertex goal)
{
    typedef float cost;
    startTimer();
    vector<Graph::vertex_descriptor> p(boost::num_vertices(g));
    vector<cost> d(boost::num_vertices(g));
    try {
        // call astar named parameter interface
        boost::astar_search
            (g, start,
             distance_heuristic<Graph, cost, vector<location> >
             (g_locations, goal),
             boost::predecessor_map(&p[0]).distance_map(&d[0]).
             visitor(astar_goal_visitor<Vertex>(goal)));
    } catch (found_goal fg) { // found a path to the goal
        endTimer();

        deque<Vertex> shortest_path;
        for(Vertex v = goal;; v = p[v]) {
            shortest_path.push_front(v);
            if(p[v] == v)
                break;
        }
        printf("### A*\n");
        printRoute(shortest_path);
        //deque<Vertex>::iterator spi = shortest_path.begin();
        //for (++spi; spi != shortest_path.end(); ++spi)
        //    cout << *spi << endl;
        //cout << "Total travel time: " << d[goal] << endl;
    }
    printTimer();
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        printf("Usage: %s START GOAL\n", argv[0]);
        exit(1);
    }
    const Graph g = loadGraph(GR_FILENAME);
    const Vertex start = atoi(argv[1]); // 開始地点
    const Vertex goal = atoi(argv[2]); // 目的地

    printf("Graph loaded: vertices: %d, edges: %d\n", boost::num_vertices(g), boost::num_edges(g));
    printf("START: %d, GOAL: %d\n", start, goal);

    dijkstra(g, start, goal);

    loadCo(CO_FILENAME);

    astar(g, start, goal);

    return 0;
}
