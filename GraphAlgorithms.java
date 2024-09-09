import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;

public class GraphAlgorithms {

    public static <T> List<Vertex<T>> bfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Input is null.");
        }
        if (!graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("Start doesn't exist in graph.");
        }
        Set<Vertex<T>> visited = new HashSet<>();
        Queue<Vertex<T>> queue = new LinkedList<>();
        List<Vertex<T>> traversal = new ArrayList<>();
        queue.add(start);
        visited.add(start);
        while (!queue.isEmpty()) {
            Vertex<T> curr = queue.remove();
            traversal.add(curr);
            List<VertexDistance<T>> adjacentList = graph.getAdjList().get(curr);
            // for each adjacent vertex, if not visited, add to queue + visited
            for (int i = 0; i < adjacentList.size(); i++) {
                VertexDistance<T> adj = adjacentList.get(i);
                Vertex<T> adjacent = adj.getVertex();
                if (!visited.contains(adjacent)) {
                    visited.add(adjacent);
                    queue.add(adjacent);
                }
            }
        }
        return traversal;
    }

    public static <T> List<Vertex<T>> dfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Input is null");
        }
        if (!graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("Start doesn't exist in graph.");
        }
        Set<Vertex<T>> visited = new HashSet<>();
        List<Vertex<T>> traversal = new ArrayList<>();
        dfsHelper(start, graph, visited, traversal);
        return traversal;
    }

    private static <T> void dfsHelper(Vertex<T> curr, Graph<T> graph,
                                      Set<Vertex<T>> visited, List<Vertex<T>> traversal) {
        visited.add(curr);
        traversal.add(curr);
        List<VertexDistance<T>> adjacentList = graph.getAdjList().get(curr);
        // for each adjacent vertex, if not visited, add to queue + visited
        for (int i = 0; i < adjacentList.size(); i++) {
            VertexDistance<T> adj = adjacentList.get(i);
            Vertex<T> adjacent = adj.getVertex();
            if (!visited.contains(adjacent)) {
                dfsHelper(adjacent, graph, visited, traversal);
            }
        }
    }

    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start,
                                                        Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Input is null.");
        }
        if (!graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("Start doesn't exist in graph.");
        }
        Set<Vertex<T>> visited = new HashSet<>();
        PriorityQueue<VertexDistance<T>> pq = new PriorityQueue<>();
        Map<Vertex<T>, Integer> distances = new HashMap<>();
        for (Vertex<T> vertex : graph.getVertices()) {
            distances.put(vertex, Integer.MAX_VALUE);
        }
        distances.put(start, 0);
        pq.add(new VertexDistance<>(start, 0));
        while (!pq.isEmpty() && (visited.size() < graph.getVertices().size())) {
            VertexDistance<T> pair = pq.poll();
            Vertex<T> vertex = pair.getVertex();
            if (!visited.contains(vertex)) {
                visited.add(vertex);
                for (VertexDistance<T> adj : graph.getAdjList().get(vertex)) {
                    Vertex<T> adjVertex = adj.getVertex();
                    int currDistance = distances.get(vertex);
                    int newDistance = currDistance + adj.getDistance();
                    if (!visited.contains(adjVertex) && newDistance < distances.get(adjVertex)) {
                        distances.put(adjVertex, newDistance);
                        pq.add(new VertexDistance<>(adjVertex, newDistance));
                    }
                }
            }
        }
        return distances;
    }

    public static <T> Set<Edge<T>> kruskals(Graph<T> graph) {
        if (graph == null) {
            throw new IllegalArgumentException("Input is null.");
        }
        Set<Edge<T>> mst = new HashSet<>();
        PriorityQueue<Edge<T>> pq = new PriorityQueue<>(graph.getEdges());
        DisjointSet<Vertex<T>> disjoint = new DisjointSet<>();
        while (mst.size() < (2 * (graph.getVertices().size() - 1))) {
            // remove edge w smallest weight
            Edge<T> edge = pq.poll();
            if (!disjoint.find(edge.getU()).equals(disjoint.find(edge.getV()))) {
                disjoint.union(edge.getU(), edge.getV());
                mst.add(edge);
                mst.add(new Edge<>(edge.getV(), edge.getU(), edge.getWeight()));
            }
        }
        if (mst.size() == (2 * (graph.getVertices().size() - 1))) {
            return mst;
        } else {
            return null;
        }
    }
}
