//Brandon Kmiec
//A2Part1: Dijkstra's Algorithm

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class A2Part1 {

    public static void main(String[] args) {
        // TODO: 11/7/2023 change "graphData.txt" to be args[0] before submitting
//        ArrayList<String> graphData = graphFromFile(args[0]);
        ArrayList<String> graphData = graphFromFile("graphData.txt");
        Graph graph = new Graph(initializeGraph(graphData));
        populateGraph(graph, graphData);
        Dijkstra dijkstra = new Dijkstra(graph);

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

    private static int initializeGraph(ArrayList<String> graphData) {
        int maxVertexValue = -1;
        String raw, vertex1, vertex2;
        StringTokenizer st;

        for (String data : graphData) {
            raw = data;
            st = new StringTokenizer(raw, " ");
            vertex1 = st.nextToken();
            vertex2 = st.nextToken();
            if (Integer.parseInt(vertex1) > maxVertexValue)
                maxVertexValue = Integer.parseInt(vertex1);
            if (Integer.parseInt(vertex2) > maxVertexValue)
                maxVertexValue = Integer.parseInt(vertex2);
        }
        return maxVertexValue + 1;
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


class Dijkstra {
    private Graph graph;

    public Dijkstra(Graph graph) {
        this.graph = graph;
    }

    public void runDijkstra() {

    }

    private void outputTable() {

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
