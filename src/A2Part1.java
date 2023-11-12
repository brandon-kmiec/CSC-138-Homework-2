//Brandon Kmiec
//A2Part1: Dijkstra's Algorithm

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class A2Part1 {

    public static void main(String[] args) {
        ArrayList<String> graphData = graphFromFile(args[0]);
        int sourceNode = Integer.parseInt(args[1]);

        Set<Integer> nodes;
        nodes = setOfNodes(graphData);

        int maxNodeValue = 0;
        for (int node : nodes) {
            if (node > maxNodeValue)
                maxNodeValue = node;
        }
        int numVertices = maxNodeValue + 1;

        Graph graph = new Graph(numVertices);
        populateGraph(graph, graphData);

        Dijkstra dijkstra = new Dijkstra(graph, nodes, sourceNode);
        dijkstra.runDijkstra();
    }

    private static ArrayList<String> graphFromFile(String fileName) {
        ArrayList<String> fileContents = new ArrayList<>();

        try {
            File file = new File(fileName);
            Scanner reader = new Scanner(file);
            while (reader.hasNextLine()) {
                fileContents.add(reader.nextLine());
            }
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        return fileContents;
    }

    private static Set<Integer> setOfNodes(ArrayList<String> graphData) {
        String raw;
        StringTokenizer st;
        Set<Integer> nodes = new HashSet<>();

        for (String data : graphData) {
            raw = data;
            st = new StringTokenizer(raw, " ");
            nodes.add(Integer.parseInt(st.nextToken()));
            nodes.add(Integer.parseInt(st.nextToken()));
        }
        return nodes;
    }

    private static void populateGraph(Graph graph, ArrayList<String> graphData) {
        String raw;
        int vertex1, vertex2, weight;
        StringTokenizer st;

        for (String data : graphData) {
            raw = data;
            st = new StringTokenizer(raw, " ");
            vertex1 = Integer.parseInt(st.nextToken());
            vertex2 = Integer.parseInt(st.nextToken());
            weight = Integer.parseInt(st.nextToken());
            graph.addEdge(vertex1, vertex2, weight);
        }
    }
}


// Pseudocode
//      Initialization:
//        1. Create a set of nodes whose least-cost-path is not definitively known
//        2. For all nodes, if the node is adjacent to the sourceNode, set the cost equal to the weight;
//           else set to Integer.MAX_VALUE
//      3. Find a node in the set of nodes with unknown least-cost-path such that the current least cost estimate of
//         the node is a minimum
//      4. Remove the node from the set of nodes with unknown least-cost-path
//      5. For all nodes with unknown least-cost-path: if the node with the min cost is adjacent to a node with unknown
//         least-cost-path, calculate the new current least-cost estimate min(current estimate of node with unknown lcp,
//         current lcp estimate of node with min cost + weight between both nodes)
class Dijkstra {
    private Graph graph;
    private int sourceNode;
    private int[] leastCostEstimate; // current estimate of cost of least-cost-path from source to destination
    private int[] predecessorNode; // predecessor node along path from source to destination
    private Set<Integer> nodes;
    private Set<Integer> nodesUnknownLCP; // set of nodes whose least-cost-path is not definitively known

    public Dijkstra(Graph graph, Set<Integer> nodes, int sourceNode) {
        this.graph = graph;
        this.nodes = nodes;
        this.sourceNode = sourceNode;

        nodesUnknownLCP = new HashSet<>(nodes);
        nodesUnknownLCP.remove(sourceNode);

        leastCostEstimate = new int[nodes.size()];
        predecessorNode = new int[nodes.size()];
    }

    public void runDijkstra() {
        initialize();

        while (!nodesUnknownLCP.isEmpty()) {
            int minNodeCost = Integer.MAX_VALUE;
            int nodeWithMinCost = -1;
            for (int node : nodesUnknownLCP) {
                if (leastCostEstimate[node] <= minNodeCost) {
                    minNodeCost = leastCostEstimate[node];
                    nodeWithMinCost = node;
                }
            }
            nodesUnknownLCP.remove(nodeWithMinCost);

            for (int node : nodesUnknownLCP) {
                if (graph.isAdjacent(node, nodeWithMinCost)) {
                    int min = Math.min(leastCostEstimate[node], leastCostEstimate[nodeWithMinCost] +
                            graph.getWeight(node, nodeWithMinCost));
                    if (leastCostEstimate[node] > min)
                        predecessorNode[node] = nodeWithMinCost;

                    leastCostEstimate[node] = min;
                }
            }
        }

        outputTable();
    }

    private void initialize() {
        for (int node : nodesUnknownLCP) {
            if (graph.isAdjacent(node, sourceNode)) {
                leastCostEstimate[node] = graph.getWeight(node, sourceNode);
                predecessorNode[node] = sourceNode;
            } else {
                leastCostEstimate[node] = Integer.MAX_VALUE;
            }
        }
    }

    private void outputTable() {
        for (int node : nodes) {
            String output = "" + leastCostEstimate[node];
            if (node != sourceNode) {
                int currentNode = node;
                while (currentNode != sourceNode) {
                    output = currentNode + " " + output;
                    currentNode = predecessorNode[currentNode];
                }
                System.out.println(sourceNode + " " + output);
            }
        }
    }
}


class Graph {
    //Fields
    private int[][] g;  //Use adjacency matrix
    private int v;  //Number of vertices
    private int e;  //Number of edges

    //Constructor
    public Graph(int v) {
        g = new int[v][v];
        this.v = v;
        e = 0;
    }//end constructor

    //Methods
    public int getNumVertices() {
        return v;
    }//end v

    public int getNumEdges() {
        return e;
    }//end e

    public void addEdge(int v1, int v2, int weight) {
        g[v1][v2] = weight;
        g[v2][v1] = weight;
        e++;
    }//end addEdge

    public boolean isAdjacent(int v1, int v2) {
        return g[v1][v2] > 0;
    }//end isAdjacent

    public boolean isConnected(int v1, int v2) {
        int[] bfs = BFStoArray(v1);
        for (int i = 0; i < bfs.length; i++)
            if (bfs[i] == v2)
                return true;
        return false;
    }//end isConnected

    public int getWeight(int v1, int v2) {
        if (isAdjacent(v1, v2))
            return g[v1][v2];
        return -1;
    }

    public int[] adj(int v) {
        ArrayList<Integer> al = new ArrayList<Integer>();
        for (int i = 0; i < this.v; i++)
            if (g[v][i] > 0)
                al.add(i);

        int[] ret = new int[al.size()];
        for (int i = 0; i < ret.length; i++)
            ret[i] = al.get(i);
        return ret;
    }//end adj

    public String adjString(int v) {
        int[] a = adj(v);
        String ret = "Verts adjacent to " + v + ": ";
        for (int i = 0; i < a.length; i++)
            ret += a[i] + " ";
        return ret;
    }//end adjString

    //Helper function that can be used later by isConnected as well
    private int[] BFStoArray(int v) {
        ArrayList<Integer> al = new ArrayList<Integer>();
        boolean[] visited = new boolean[this.v];
        Queue<Integer> q = new LinkedList<Integer>();
        q.add(v);
        visited[v] = true;
        int visit;
        while (!q.isEmpty()) {
            visit = q.remove();
            al.add(visit);
            for (int i = 0; i < this.v; i++)
                if (isAdjacent(visit, i) && !visited[i]) {
                    q.add(i);
                    visited[i] = true;
                }//end if
        }//end while
        int[] ret = new int[al.size()];
        for (int i = 0; i < ret.length; i++)
            ret[i] = al.get(i);
        return ret;
    }//end BFStoArray

    public String BFS(int v) {
        String ret = "BFS for vert #" + v + ": ";
        int[] bfs = BFStoArray(v);
        for (int i = 0; i < bfs.length; i++)
            ret += bfs[i] + " ";
        return ret;
    }//end BFS

    //Iterator (wrapper) method for recursive call
    public String DFS(int v) {
        boolean[] visited = new boolean[this.v];
        String ret = DFS(v, visited, "DFS for vert #" + v + ": ");
        return ret;
    }//end DFS

    //Workhorse (helper) method for recursive call
    private String DFS(int v, boolean[] visited, String str) {
        str += v + " ";
        visited[v] = true;
        for (int i = 0; i < this.v; i++)
            if (isAdjacent(v, i) && !visited[i])
                str = DFS(i, visited, str);
        return str;
    }//end DFS
}//end class Graph
