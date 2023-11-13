//Brandon Kmiec
//A2Part3: Bellman Ford Algorithm and Dynamic Programming

import jdk.jfr.Unsigned;

import java.io.File;
import java.io.FileNotFoundException;
import java.math.BigInteger;
import java.util.*;

public class A2Part3 {
    public static void main(String[] args) {
//        ArrayList<String> graphData = graphFromFile(args[0]);
//        int sourceNode = Integer.parseInt(args[1]);

        ArrayList<String> graphData = graphFromFile("graphData.txt");
        int sourceNode = Integer.parseInt("2");

        Set<Integer> nodes;
        nodes = setOfNodes(graphData);

        int maxNodeValue = 0;
        for (int node : nodes) {
            if (node > maxNodeValue)
                maxNodeValue = node;
        }
        int numVertices = maxNodeValue + 1;

        GraphP3 graph = new GraphP3(numVertices);
        populateGraph(graph, graphData);

        BellmanFord bellmanFord = new BellmanFord(graph, sourceNode, nodes);
        bellmanFord.runBellmanFord();
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

    private static void populateGraph(GraphP3 graph, ArrayList<String> graphData) {
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


class BellmanFord {
    private GraphP3 graph;
    private int sourceNode;
    private Set<Integer> nodes;
    private int[] distance;
    private int[] predecessorNode;


    public BellmanFord(GraphP3 graph, int sourceNode, Set<Integer> nodes) {
        this.graph = graph;
        this.sourceNode = sourceNode;
        this.nodes = nodes;

        distance = new int[nodes.size()];
        predecessorNode = new int[nodes.size()];
    }

    public void runBellmanFord() {
        initialize();

//        for (int i = 0; i < nodes.size(); i++) {
//            for (int j = 0; j < nodes.size(); j++) {
//                if (graph.isAdjacent(i, j)) {
//                    int min = Math.min(distance[j], distance[i] + graph.getWeight(i, j));
//                    if (distance[j] > min) {
//                        predecessorNode[j] = i;
//                    }
//                    distance[j] = min;
//                }
//            }
//        }

//        for (int node : nodes) {
//            for (int adjNode : graph.adj(node)) {
//                int min = Math.min(distance[adjNode], distance[node] + graph.getWeight(adjNode, node));
//                if (distance[adjNode] > min) {
//                    predecessorNode[adjNode] = node;
//                }
//                distance[adjNode] = min;
//            }
//        }
        for (int iteration = 0; iteration < graph.getNumVertices() - 1; iteration++)
            for (int i = 0; i < graph.getNumVertices(); i++) {
                for (int j = 0; j < graph.getNumEdges() - 2; j++) {
                    if (graph.isAdjacent(i, j)) {
                        if (distance[j] != Integer.MAX_VALUE && distance[j] + graph.getWeight(i, j) < distance[i]) {
                            distance[i] = distance[j] + graph.getWeight(i, j);
                            predecessorNode[i] = j;
                        }
                    }
                }
            }

        output();
    }

    private void initialize() {
        for (int node : nodes) {
            if (node == sourceNode)
                distance[node] = 0;
            else
                distance[node] = Integer.MAX_VALUE;
            predecessorNode[node] = -1;
        }
    }

    private void output() {
        for (int node : nodes) {
            String output = "" + distance[node];
            if (node != sourceNode) {
                int currentNode = node;
                while (currentNode != sourceNode && currentNode != -1) {
                    output = currentNode + " " + output;
                    currentNode = predecessorNode[currentNode];
                }
                System.out.println(sourceNode + " " + output);
            } else
                System.out.println(sourceNode + " " + sourceNode + " " + 0);
        }
    }
}


class GraphP3 {
    //Fields
    private int[][] g;  //Use adjacency matrix
    private int v;  //Number of vertices
    private int e;  //Number of edges
    private Set<Integer> visited;

    //Constructor
    public GraphP3(int v) {
        g = new int[v][v];
        this.v = v;
        e = 0;
        visited = new HashSet<>();
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

    public void removeEdge(int v1, int v2) {
        g[v1][v2] = 0;
        g[v2][v1] = 0;
        e--;
    }

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

    // Pseudocode
    //      for every vertex that has not been visited: if cycleDFS of the current vertex is true, clear visited for
    //      future use and return true.  Clears visited for future use and returns false as default.
    public boolean containsCycle() {
        for (int i = 0; i < this.v; i++) {
            if (!visited.contains(i))
                if (cycleDFS(i, -1)) {
                    visited.clear();
                    return true;
                }
        }
        visited.clear();
        return false;
    }

    // Pseudocode
    //      add the current vertex to visited.  For every vertex that is adjacent to the current vertex and not equal
    //      to the parent vertex: return true if the vertex has been visited or if the recursive call to cycleDFS with
    //      new current vertex = vertex and new parent vertex = i returns true.  Return false as default.
    private boolean cycleDFS(int i, int parent) {
        visited.add(i);
        for (int vertex = 0; vertex < this.v; vertex++) {
            if (isAdjacent(i, vertex) && vertex != parent) {
                if (visited.contains(vertex) || cycleDFS(vertex, i))
                    return true;
            }
        }
        return false;
    }
}