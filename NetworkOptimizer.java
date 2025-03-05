// Question No 5

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;

/*
Network Optimizer Application
GUI application for designing network topologies with cost and latency optimization.
Features visual graph editing, minimum spanning tree calculation, and shortest path finding.
*/
public class NetworkOptimizer extends JFrame {
    // GUI Components
    private JPanel graphPanel;              // Main drawing area for network visualization
    private JButton addNodeButton,          // Button to create new nodes
                   addEdgeButton,          // Button to create new connections
                   optimizeButton,         // Button to calculate MST
                   findPathButton;         // Button to find shortest path
    private JTextField costField,           // Input for connection cost
                     bandwidthField;       // Input for connection bandwidth
    private JLabel totalCostLabel,          // Displays total network cost
                 latencyLabel;            // Displays path latency

    // Network Data Structures
    private List<Node> nodes = new ArrayList<>();    // All nodes in the network
    private List<Edge> edges = new ArrayList<>();    // All potential connections
    private List<Edge> mstEdges = new ArrayList<>(); // Edges in Minimum Spanning Tree
    private List<Edge> shortestPathEdges = new ArrayList<>(); // Edges in shortest path
    private Graph graph;                            // Graph algorithm processor

    
    //Initializes the GUI layout and components
    
    public NetworkOptimizer() {
        // Window Configuration
        setTitle("Network Optimizer");
        setSize(800, 600);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new BorderLayout());

        // Initialize Network Visualization Panel
        graphPanel = new GraphPanel();
        add(graphPanel, BorderLayout.CENTER);

        // Control Panel Setup
        JPanel controlPanel = new JPanel();
        addNodeButton = new JButton("Add Node");
        addEdgeButton = new JButton("Add Edge");
        optimizeButton = new JButton("Optimize Network");
        findPathButton = new JButton("Find Shortest Path");
        costField = new JTextField(5);
        bandwidthField = new JTextField(5);
        totalCostLabel = new JLabel("Total Cost: 0");
        latencyLabel = new JLabel("Latency: N/A");

        // Add Components to Control Panel
        controlPanel.add(addNodeButton);
        controlPanel.add(addEdgeButton);
        controlPanel.add(new JLabel("Cost:"));
        controlPanel.add(costField);
        controlPanel.add(new JLabel("Bandwidth:"));
        controlPanel.add(bandwidthField);
        controlPanel.add(optimizeButton);
        controlPanel.add(findPathButton);
        controlPanel.add(totalCostLabel);
        controlPanel.add(latencyLabel);
        
        // Add Control Panel to Window
        add(controlPanel, BorderLayout.SOUTH);

        // Event Listeners
        addNodeButton.addActionListener(e -> addNode());
        addEdgeButton.addActionListener(e -> addEdge());
        optimizeButton.addActionListener(e -> optimizeNetwork());
        findPathButton.addActionListener(e -> findShortestPath());

        setVisible(true);  // Display the window
    }

    
    //Represents a network node (server/client) with screen coordinates
    
    static class Node {
        int id;     // Unique identifier
        int x, y;   // Screen position
        
        Node(int id, int x, int y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }
    }

    
    //Represents a network connection between two nodes
    
    static class Edge {
        Node src, dest;   // Connected nodes
        int cost;        // Connection cost
        int bandwidth;    // Data capacity
        
        Edge(Node src, Node dest, int cost, int bandwidth) {
            this.src = src;
            this.dest = dest;
            this.cost = cost;
            this.bandwidth = bandwidth;
        }

        // Edge comparison considering both directions
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof Edge)) return false;
            Edge other = (Edge) obj;
            return (src.id == other.src.id && dest.id == other.dest.id) ||
                   (src.id == other.dest.id && dest.id == other.src.id);
        }

        @Override
        public int hashCode() {
            return Objects.hash(Math.min(src.id, dest.id), Math.max(src.id, dest.id));
        }
    }

    
    //Custom panel for network visualization
     
    class GraphPanel extends JPanel {
        /*
         Renders network components with different visual styles:
         Regular edges: Black thin lines
         MST edges: Green thick lines
         Shortest path: Red thick lines
         Nodes: Blue circles with labels
        */
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2d = (Graphics2D) g;
            
            // Draw all network connections
            for (Edge edge : edges) {
                // Set visual style based on edge role
                if (shortestPathEdges.contains(edge)) {
                    g2d.setColor(Color.RED);
                    g2d.setStroke(new BasicStroke(3));
                } else if (mstEdges.contains(edge)) {
                    g2d.setColor(Color.GREEN);
                    g2d.setStroke(new BasicStroke(3));
                } else {
                    g2d.setColor(Color.BLACK);
                    g2d.setStroke(new BasicStroke(1));
                }
                
                // Draw connection line
                g2d.drawLine(edge.src.x, edge.src.y, edge.dest.x, edge.dest.y);
                
                // Display cost/bandwidth at midpoint
                int midX = (edge.src.x + edge.dest.x) / 2;
                int midY = (edge.src.y + edge.dest.y) / 2;
                g2d.drawString("C:" + edge.cost + " B:" + edge.bandwidth, midX, midY);
            }

            // Draw all nodes
            for (Node node : nodes) {
                g2d.setColor(Color.BLUE);
                g2d.fillOval(node.x - 10, node.y - 10, 20, 20);  // Node circle
                g2d.setColor(Color.BLACK);
                g2d.drawString("Node " + node.id, node.x - 15, node.y - 15); // Label
            }
        }
    }

    
    //Creates a new node at random position
     
    private void addNode() {
        int id = nodes.size();
        int x = 50 + (int)(Math.random() * 700);  // Random X within panel
        int y = 50 + (int)(Math.random() * 500);  // Random Y within panel
        nodes.add(new Node(id, x, y));
        graphPanel.repaint();  // Refresh display
    }

    
    //Creates connection between last two nodes with specified parameters
    
    private void addEdge() {
        if (nodes.size() < 2) {
            JOptionPane.showMessageDialog(this, "Need at least two nodes to add an edge.");
            return;
        }
        try {
            // Parse input values
            int cost = Integer.parseInt(costField.getText());
            int bandwidth = Integer.parseInt(bandwidthField.getText());
            
            // Connect last two created nodes
            Node src = nodes.get(nodes.size() - 2);
            Node dest = nodes.get(nodes.size() - 1);
            edges.add(new Edge(src, dest, cost, bandwidth));
            
            graphPanel.repaint();  // Refresh display
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this, "Invalid cost or bandwidth.");
        }
    }

    /*
     Calculates Minimum Spanning Tree using Kruskal's algorithm
     Updates display with optimal network connections (green)
     */
    private void optimizeNetwork() {
        if (nodes.isEmpty()) return;
        
        // Initialize graph structure
        graph = new Graph(nodes.size());
        for (Edge edge : edges) {
            graph.addEdge(edge.src.id, edge.dest.id, edge.cost);
        }
        
        // Calculate MST
        Graph.MSTResult result = graph.kruskalMST();
        if (result.totalCost == -1) {
            JOptionPane.showMessageDialog(this, "Network is not connected.");
            return;
        }
        
        // Update UI with results
        totalCostLabel.setText("Total Cost: " + result.totalCost);
        mstEdges.clear();
        
        // Map algorithm edges to visual edges
        for (Graph.Edge mstEdge : result.mstEdges) {
            for (Edge guiEdge : edges) {
                if (guiEdge.equals(new Edge(new Node(mstEdge.src, 0, 0), 
                        new Node(mstEdge.dest, 0, 0), 0, 0))) {
                    mstEdges.add(guiEdge);
                    break;
                }
            }
        }
        graphPanel.repaint();
    }

    /*
     Finds shortest path using Dijkstra's algorithm with bandwidth weights
     Updates display with optimal path (red)
     */
    private void findShortestPath() {
        if (nodes.size() < 2) {
            JOptionPane.showMessageDialog(this, "Need at least two nodes.");
            return;
        }
        
        // Use first and last nodes as default path endpoints
        int srcId = nodes.get(0).id;
        int destId = nodes.get(nodes.size() - 1).id;
        
        // Initialize graph with inverse bandwidth weights
        graph = new Graph(nodes.size());
        for (Edge edge : edges) {
            graph.addEdge(edge.src.id, edge.dest.id, 1.0 / edge.bandwidth);
        }
        
        // Calculate shortest path
        Graph.DijkstraResult result = graph.dijkstra(srcId, destId);
        if (result.latency == Double.POSITIVE_INFINITY) {
            JOptionPane.showMessageDialog(this, "No path exists.");
            return;
        }
        
        // Update UI with results
        latencyLabel.setText("Latency: " + String.format("%.2f", result.latency));
        shortestPathEdges.clear();
        
        // Map algorithm path to visual edges
        List<Integer> path = result.path;
        for (int i = 0; i < path.size() - 1; i++) {
            int u = path.get(i);
            int v = path.get(i + 1);
            for (Edge edge : edges) {
                if ((edge.src.id == u && edge.dest.id == v) || 
                    (edge.src.id == v && edge.dest.id == u)) {
                    shortestPathEdges.add(edge);
                    break;
                }
            }
        }
        graphPanel.repaint();
    }

    
    //Graph processing engine containing algorithms
     
    static class Graph {
        private int V;                   // Number of vertices
        private List<Edge> edges = new ArrayList<>();  // Connection list

        
        //Internal edge representation for algorithms
         
        class Edge implements Comparable<Edge> {
            int src, dest;    // Connected nodes
            double weight;     // Algorithm weight
            
            Edge(int src, int dest, double weight) {
                this.src = src;
                this.dest = dest;
                this.weight = weight;
            }

            // For sorting edges by weight
            @Override
            public int compareTo(Edge other) {
                return Double.compare(this.weight, other.weight);
            }
        }

        Graph(int V) {
            this.V = V;
        }

        
        //Adds a weighted edge to the graph
         
        void addEdge(int src, int dest, double weight) {
            edges.add(new Edge(src, dest, weight));
        }

        /*
          Kruskal's Minimum Spanning Tree Algorithm
          Returns total cost and selected edges
         */
        MSTResult kruskalMST() {
            // Sort edges by ascending weight
            Collections.sort(edges);
            
            UnionFind uf = new UnionFind(V);  // Cycle detection
            List<Edge> mstEdges = new ArrayList<>();
            int totalCost = 0;
            int edgesAdded = 0;

            // Greedily select smallest edges that don't form cycles
            for (Edge edge : edges) {
                int rootSrc = uf.find(edge.src);
                int rootDest = uf.find(edge.dest);
                if (rootSrc != rootDest) {
                    uf.union(rootSrc, rootDest);
                    mstEdges.add(edge);
                    totalCost += (int) edge.weight;  // Cast to int for cost display
                    edgesAdded++;
                    if (edgesAdded == V - 1) break;  // MST complete
                }
            }
            
            return edgesAdded == V - 1 ? 
                new MSTResult(totalCost, mstEdges) : 
                new MSTResult(-1, null);  // -1 indicates disconnected graph
        }

        /*
          Dijkstra's Shortest Path Algorithm
          Returns latency and node path
         */
        DijkstraResult dijkstra(int src, int dest) {
            // Build adjacency list
            Map<Integer, List<Edge>> adj = new HashMap<>();
            for (int i = 0; i < V; i++) adj.put(i, new ArrayList<>());
            for (Edge edge : edges) {
                // Add bidirectional connections
                adj.get(edge.src).add(new Edge(edge.src, edge.dest, edge.weight));
                adj.get(edge.dest).add(new Edge(edge.dest, edge.src, edge.weight));
            }

            // Distance array initialization
            double[] dist = new double[V];
            Arrays.fill(dist, Double.POSITIVE_INFINITY);
            dist[src] = 0;

            int[] prev = new int[V];  // Path reconstruction
            Arrays.fill(prev, -1);

            // Priority queue for efficient minimum extraction
            PriorityQueue<NodeDist> pq = new PriorityQueue<>(Comparator.comparingDouble(n -> n.dist));
            pq.add(new NodeDist(src, 0));

            // Main algorithm loop
            while (!pq.isEmpty()) {
                NodeDist current = pq.poll();
                if (current.node == dest) break;  // Early exit
                if (current.dist > dist[current.node]) continue;  // Stale entry

                // Explore neighbors
                for (Edge edge : adj.get(current.node)) {
                    double newDist = dist[current.node] + edge.weight;
                    if (newDist < dist[edge.dest]) {
                        dist[edge.dest] = newDist;
                        prev[edge.dest] = current.node;
                        pq.add(new NodeDist(edge.dest, newDist));
                    }
                }
            }

            // Handle no path found
            if (dist[dest] == Double.POSITIVE_INFINITY) {
                return new DijkstraResult(Double.POSITIVE_INFINITY, null);
            }

            // Reconstruct path
            LinkedList<Integer> path = new LinkedList<>();
            for (int at = dest; at != -1; at = prev[at]) path.addFirst(at);
            return new DijkstraResult(dist[dest], path);
        }

        /*
          Union-Find (Disjoint Set Union) data structure
          Efficient cycle detection for Kruskal's
         */
        class UnionFind {
            int[] parent, rank;
            
            UnionFind(int n) {
                parent = new int[n];
                rank = new int[n];
                for (int i = 0; i < n; i++) parent[i] = i;
            }
            
            // Path compression
            int find(int x) {
                if (parent[x] != x) parent[x] = find(parent[x]);
                return parent[x];
            }
            
            // Union by rank
            void union(int x, int y) {
                int xRoot = find(x), yRoot = find(y);
                if (xRoot == yRoot) return;
                if (rank[xRoot] < rank[yRoot]) parent[xRoot] = yRoot;
                else {
                    parent[yRoot] = xRoot;
                    if (rank[xRoot] == rank[yRoot]) rank[xRoot]++;
                }
            }
        }

        
        //Helper class for Dijkstra's priority queue
         
        class NodeDist {
            int node;     // Node ID
            double dist; // Current distance
            
            NodeDist(int node, double dist) {
                this.node = node;
                this.dist = dist;
            }
        }

        // Algorithm Result Classes
        class MSTResult {
            int totalCost;
            List<Edge> mstEdges;
            MSTResult(int cost, List<Edge> edges) {
                totalCost = cost;
                mstEdges = edges;
            }
        }

        class DijkstraResult {
            double latency;
            List<Integer> path;
            DijkstraResult(double lat, List<Integer> p) {
                latency = lat;
                path = p;
            }
        }
    }

    
    //Application entry point
     
    public static void main(String[] args) {
        // Ensure GUI runs on Event Dispatch Thread
        SwingUtilities.invokeLater(NetworkOptimizer::new);
    }
}