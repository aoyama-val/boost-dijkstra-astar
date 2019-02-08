//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt

using namespace boost;
using namespace std;

// auxiliary types
struct location
{
    int y, x; // lat, long
};
typedef float cost;




// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
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


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
        void examine_vertex(Vertex u, Graph& g) {
            if(u == m_goal)
                throw found_goal();
        }
private:
    Vertex m_goal;
};

vector<location> g_locations;

void loadLocations(const char* filename)
{
    FILE* fp = fopen(filename, "r");

    if (!fp) {
        fprintf(stderr, "Cound not open: %s\n", filename);
        exit(1);
    }

    location loc;
    g_locations.push_back(loc); // dummy

    while (!feof(fp)) {
        char buf[256];
        char type[10];
        int n;
        int y;
        int x;
        fgets(buf, sizeof(buf), fp);
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

int main(int argc, char **argv)
{
    // specify some types
    typedef adjacency_list<listS, vecS, directedS, no_property, property<edge_weight_t, cost> > mygraph_t;
    typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
    typedef mygraph_t::vertex_descriptor vertex;
    typedef mygraph_t::edge_descriptor edge_descriptor;
    typedef mygraph_t::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;

    set<int> vertices;
    vector<edge> edge_array;
    vector<cost> weights;

    loadLocations("USA-road-d.NY.co");

    const char* filename = "USA-road-d.NY.gr";
    FILE* fp = fopen(filename, "r");

    if (!fp) {
        fprintf(stderr, "Cound not open: %s\n", filename);
        exit(1);
    }

    while (!feof(fp)) {
        char buf[256];
        char type[10];
        int from;
        int to;
        int weight;
        fgets(buf, sizeof(buf), fp);
        if (buf[0] == 'a') {
            sscanf(buf, "%s %d %d %d", type, &from, &to, &weight);
            vertices.insert(from);
            vertices.insert(to);
            edge_array.push_back(std::make_pair(from, to));
            weights.push_back(weight);
        }
    }

    fclose(fp);

    const int N = vertices.size();
    // create graph
    mygraph_t g(N);
    WeightMap weightmap = get(edge_weight, g);
    unsigned int num_edges = edge_array.size();
    for(std::size_t j = 0; j < num_edges; ++j) {
        edge_descriptor e; bool inserted;
        tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
        weightmap[e] = weights[j];
    }


    vertex start = 91;
    vertex goal = 41;

    cout << "Start vertex: " << start << endl;
    cout << "Goal vertex: " << goal << endl;

    vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
    vector<cost> d(num_vertices(g));
    struct timespec tsStart;
    struct timespec tsEnd;
    try {
        clock_gettime(CLOCK_REALTIME, &tsStart);

        // call astar named parameter interface
        astar_search
            (g, start,
             distance_heuristic<mygraph_t, cost, vector<location> >
             (g_locations, goal),
             predecessor_map(&p[0]).distance_map(&d[0]).
             visitor(astar_goal_visitor<vertex>(goal)));
    } catch(found_goal fg) { // found a path to the goal
        clock_gettime(CLOCK_REALTIME, &tsEnd);
        list<vertex> shortest_path;
        for(vertex v = goal;; v = p[v]) {
            shortest_path.push_front(v);
            if(p[v] == v)
                break;
        }
        cout << "Shortest path from " << start << " to "
            << goal << ": " << endl;
        list<vertex>::iterator spi = shortest_path.begin();
        cout << start << endl;
        for (++spi; spi != shortest_path.end(); ++spi)
            cout << *spi << endl;
        cout << endl << "Total travel time: " << d[goal] << endl;
        printf("Time: %g sec\n", (double)(tsEnd.tv_sec - tsStart.tv_sec) + 1.0e-9 * (tsEnd.tv_nsec - tsStart.tv_nsec));
        return 0;
    }

    cout << "Didn't find a path from " << start << "to"
        << goal << "!" << endl;
    return 0;

}
