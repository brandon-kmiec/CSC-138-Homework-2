//Brandon Kmiec
//A2Part2: Kruskal's Algorithm

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class A2Part2 {
    public static void main(String[] args) {
        ArrayList<String> graphData = graphFromFile(args[0]);

        Set<Integer> nodes;
        nodes = setOfNodes(graphData);

        int maxNodeValue = 0;
        for (int node : nodes) {
            if (node > maxNodeValue)
                maxNodeValue = node;
        }//end for
        int numVertices = maxNodeValue + 1;

        GraphP2 graph = new GraphP2(numVertices);
        populateGraph(graph, graphData);

        Kruskal kruskal = new Kruskal(graph, graphData);
        kruskal.runKruskal();
    }//end main

    private static ArrayList<String> graphFromFile(String fileName) {
        ArrayList<String> fileContents = new ArrayList<>();

        try {
            File file = new File(fileName);
            Scanner reader = new Scanner(file);
            while (reader.hasNextLine())
                fileContents.add(reader.nextLine());
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }//end try-catch

        return fileContents;
    }//end graphFromFile

    private static Set<Integer> setOfNodes(ArrayList<String> graphData) {
        String raw;
        StringTokenizer st;
        Set<Integer> nodes = new HashSet<>();

        for (String data : graphData) {
            raw = data;
            st = new StringTokenizer(raw, " ");
            nodes.add(Integer.parseInt(st.nextToken()));
            nodes.add(Integer.parseInt(st.nextToken()));
        }//end for
        return nodes;
    }//end setOfNodes

    private static void populateGraph(GraphP2 graph, ArrayList<String> graphData) {
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
        }//end for
    }//end populateGraph
}//end A2Part2


// Algorithm
//      1. Sort all edges in increasing order of their weight
//      2. Pick the edge with the smallest weight. Include the edge if a cycle is not formed.
//         If a cycle is formed, discard it.
//      3. Repeat step 2 until every edge has been checked.
class Kruskal {
    private GraphP2 graph;
    private ArrayList<String> graphData;
    private int[] edgeWeights;
    private int[] node1;
    private int[] node2;
    private int totalWeight;
    private ArrayList<Integer> edgeWeightsAL, node1AL, node2AL;
    private GraphP2 resultGraph;

    public Kruskal(GraphP2 graph, ArrayList<String> graphData) {
        this.graph = graph;
        this.graphData = graphData;

        edgeWeights = new int[graph.getNumEdges()];
        node1 = new int[graph.getNumEdges()];
        node2 = new int[graph.getNumEdges()];

        totalWeight = 0;

        edgeWeightsAL = new ArrayList<>();
        node1AL = new ArrayList<>();
        node2AL = new ArrayList<>();

        resultGraph = new GraphP2(graph.getNumVertices());
    }//end constructor

    public void runKruskal() {
        initialize();
        sortWeights();

        for (int i = 0; i < edgeWeights.length; i++) {
            resultGraph.addEdge(node1[i], node2[i], edgeWeights[i]);
            if (resultGraph.containsCycle()) {
                resultGraph.removeEdge(node1[i], node2[i]);
            } else {
                totalWeight += edgeWeights[i];
                edgeWeightsAL.add(edgeWeights[i]);
                node1AL.add(node1[i]);
                node2AL.add(node2[i]);
            }//end if-else
        }//end for

        outputTable();
    }//end runKruskal

    private void initialize() {
        String raw;
        int vertex1, vertex2, weight;
        StringTokenizer st;

        for (int i = 0; i < graphData.size(); i++) {
            raw = graphData.get(i);
            st = new StringTokenizer(raw, " ");
            vertex1 = Integer.parseInt(st.nextToken());
            vertex2 = Integer.parseInt(st.nextToken());
            weight = Integer.parseInt(st.nextToken());

            edgeWeights[i] = weight;
            node1[i] = vertex1;
            node2[i] = vertex2;
        }//end for
    }//end initialize

    private void sortWeights() {
        int tempWeight, tempNode1, tempNode2;
        for (int i = 0; i < edgeWeights.length - 1; i++) {
            for (int j = 0; j < edgeWeights.length - 1; j++) {
                if (edgeWeights[j] > edgeWeights[j + 1]) {
                    tempWeight = edgeWeights[j];
                    edgeWeights[j] = edgeWeights[j + 1];
                    edgeWeights[j + 1] = tempWeight;

                    tempNode1 = node1[j];
                    node1[j] = node1[j + 1];
                    node1[j + 1] = tempNode1;

                    tempNode2 = node2[j];
                    node2[j] = node2[j + 1];
                    node2[j + 1] = tempNode2;
                }//end if
            }//end for
        }//end for
    }//end sortWeights

    private void outputTable() {
        for (int i = 0; i < edgeWeightsAL.size(); i++)
            System.out.println(node1AL.get(i) + " " + node2AL.get(i) + " " + edgeWeightsAL.get(i));
        System.out.println(totalWeight);
    }//end outputTable
}//end Kruskal


class GraphP2 {
    //Fields
    private int[][] g;  //Use adjacency matrix
    private int v;  //Number of vertices
    private int e;  //Number of edges
    private Set<Integer> visited;

    //Constructor
    public GraphP2(int v) {
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
    }//end removeEdge

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
    }//end getWeight

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

    // Algorithm
    //      for every vertex that has not been visited: if cycleDFS of the current vertex is true, clear visited for
    //      future use and return true.  Clears visited for future use and returns false as default.
    public boolean containsCycle() {
        for (int i = 0; i < this.v; i++) {
            if (!visited.contains(i))
                if (cycleDFS(i, -1)) {
                    visited.clear();
                    return true;
                }//end if
        }//end for
        visited.clear();
        return false;
    }//end containsCycle

    // Algorithm
    //      add the current vertex to visited.  For every vertex that is adjacent to the current vertex and not equal
    //      to the parent vertex: return true if the vertex has been visited or if the recursive call to cycleDFS with
    //      new current vertex = vertex and new parent vertex = i returns true.  Return false as default.
    private boolean cycleDFS(int i, int parent) {
        visited.add(i);
        for (int vertex = 0; vertex < this.v; vertex++) {
            if (isAdjacent(i, vertex) && vertex != parent)
                if (visited.contains(vertex) || cycleDFS(vertex, i))
                    return true;
        }//end for
        return false;
    }//end cycleDFS
}//end GraphP2
