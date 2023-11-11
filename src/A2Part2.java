//Brandon Kmiec
//A2Part2: Kruskal's Algorithm

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class A2Part2 {
    public static void main(String[] args) {
        // TODO: 11/11/2023 change fileName to args[0]
        ArrayList<String> graphData = graphFromFile("graphData.txt");
//        int sourceNode = Integer.parseInt(args[1]);

        Set<Integer> nodes;
        nodes = setOfNodes(graphData);
        GraphP2 graph = new GraphP2(nodes.size());

        populateGraph(graph, graphData);
        Kruskal kruskal = new Kruskal(graph, nodes, graphData);
        kruskal.runKruskal();
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
        }
    }
}


class Kruskal {
    private GraphP2 graph;
    private Set<Integer> nodes;
    private ArrayList<String> graphData;
    private int[] edgeWeights;
    private int[] node1;
    private int[] node2;

    public Kruskal(GraphP2 graph, Set<Integer> nodes, ArrayList<String> graphData) {
        this.graph = graph;
        this.nodes = nodes;
        this.graphData = graphData;

        edgeWeights = new int[graph.getNumEdges()];
        node1 = new int[graph.getNumEdges()];
        node2 = new int[graph.getNumEdges()];
    }

    public void runKruskal() {
        initialize();
        sortWeights();

        outputTable();
    }

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
        }
    }

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
                }
            }
        }
    }

    private void outputTable() {

    }
}


class GraphP2 {
    //Fields
    private int[][] g;  //Use adjacency matrix
    private int v;  //Number of vertices
    private int e;  //Number of edges

    //Constructor
    public GraphP2(int v) {
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
