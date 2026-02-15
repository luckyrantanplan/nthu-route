# NTHU-Route Algorithm Documentation

This document describes the primary algorithms used in the NTHU-Route global router for VLSI design. NTHU-Route 2.0 won 1st place at the ISPD 2008 Global Routing Contest.

## Overall Routing Pipeline

The routing process follows a multi-stage pipeline:

```mermaid
flowchart TD
    A[Input: Netlist & Grid] --> B[FLUTE Steiner Tree Generation]
    B --> C[2D Tree Construction & Decomposition]
    C --> D[L-Pattern Routing]
    D --> E[Iterative Rip-up & Reroute]
    E --> F{Congestion Resolved?}
    F -->|No| E
    F -->|Yes| G[Layer Assignment]
    G --> H[Post-Processing & Refinement]
    H --> I[Output: Routed Design]
```

## 1. FLUTE Steiner Tree Generation

The [FLUTE](http://home.eng.iastate.edu/~cnchu/) library generates rectilinear Steiner minimum trees (RSMTs) for each net. FLUTE uses precomputed lookup tables (`POWV9.dat`, `POST9.dat`) to efficiently construct near-optimal Steiner trees.

**Purpose:** Provide an initial routing topology that minimizes total wirelength.

```mermaid
flowchart LR
    A[Net Pin Coordinates] --> B[FLUTE Algorithm]
    B --> C[Steiner Tree Topology]
    C --> D[Branch Decomposition]
    D --> E[2-Pin Net Elements]
```

## 2. Congestion-Aware Cost Function

NTHU-Route uses a history-based cost function to evaluate edge congestion. The cost function guides routing decisions away from congested areas.

```mermaid
flowchart TD
    A[Edge Capacity & Demand] --> B[Cost Function Evaluation]
    B --> C{Cost Type}
    C -->|FastRoute| D["cost = 1 + h × 1/(1 + exp(-k×(demand - capacity)))"]
    C -->|History-Based| E["cost = base_cost × history_factor"]
    D --> F[Edge Weight for Routing]
    E --> F
```

The sigmoid-based cost function increases sharply as edge demand approaches capacity, steering routes toward less congested edges.

## 3. 2D Tree Construction

This stage decomposes Steiner trees into 2-pin connections and routes them using multiple strategies:

```mermaid
flowchart TD
    A[Steiner Tree] --> B[Decompose into 2-Pin Nets]
    B --> C[Sort by Bounding Box Size]
    C --> D[L-Pattern Routing]
    D --> E{Path Found?}
    E -->|Yes| F[Update Congestion Map]
    E -->|No| G[Monotonic Routing]
    G --> H{Path Found?}
    H -->|Yes| F
    H -->|No| I[Maze Routing]
    I --> F
    F --> J[Edge Shifting Optimization]
```

### L-Pattern Routing

For each 2-pin net, the algorithm tries L-shaped paths (one bend) connecting the two pins. It evaluates both possible L-shapes and selects the one with lower congestion cost.

### Edge Shifting

After initial routing, edge shifting optimizes the Steiner tree by adjusting Steiner point positions to reduce overall congestion.

## 4. Monotonic Routing

Monotonic routing finds paths that are monotonic in at least one dimension (x or y). It uses dynamic programming to evaluate paths that only move in increasing coordinate directions.

```mermaid
flowchart TD
    A[Source Pin] --> B{Direction Check}
    B -->|X-Monotonic| C[DP: Only Move Right/Up/Down]
    B -->|Y-Monotonic| D[DP: Only Move Up/Left/Right]
    C --> E[Track Cost & Via Count]
    D --> E
    E --> F[Select Minimum Cost Path]
    F --> G[Backtrace Optimal Path]
```

**Advantages:** Efficient for non-congested regions; avoids detours.

## 5. Multi-Source Multi-Sink Maze Routing

When monotonic routing fails, the maze router finds paths using a Dijkstra-like algorithm with a priority queue (pairing heap).

```mermaid
flowchart TD
    A[Initialize Source Nodes] --> B[Priority Queue]
    B --> C[Pop Minimum Cost Node]
    C --> D{Is Destination?}
    D -->|Yes| E[Backtrace Path]
    D -->|No| F[Expand Neighbors]
    F --> G{Within Bounding Box?}
    G -->|Yes| H[Update Cost & Push]
    G -->|No| I[Skip]
    H --> B
    I --> B
    E --> J[Return Routed Path]
```

The bounding box is adaptively expanded if no path is found within the initial box. This balances between routing quality and runtime.

## 6. Range Router (Rip-up & Reroute)

The range router handles congestion by iteratively ripping up and rerouting nets in congested regions:

```mermaid
flowchart TD
    A[Identify Congested Edges] --> B[Sort by Congestion Level]
    B --> C[Group into 10 Intervals]
    C --> D[Process Most Congested First]
    D --> E[Define Routing Range/Box]
    E --> F[Rip-up Nets in Range]
    F --> G[Reroute with Updated Costs]
    G --> H{Overflow Reduced?}
    H -->|Yes| I[Next Interval]
    H -->|No| J[Expand Range]
    J --> G
    I --> K{All Intervals Done?}
    K -->|No| D
    K -->|Yes| L[Update History Costs]
```

## 7. Layer Assignment

After 2D routing, the layer assignment stage maps routes to specific metal layers in the 3D routing grid:

```mermaid
flowchart TD
    A[2D Routing Solution] --> B[Group Nets by Congestion]
    B --> C[Sort Nets by Priority]
    C --> D[K-Shortest Path Layer Assignment]
    D --> E[Dynamic Programming]
    E --> F{Via Overflow?}
    F -->|Yes| G[Try Alternative Layers]
    F -->|No| H[Assign Layer]
    G --> H
    H --> I[Update 3D Capacity]
    I --> J[Output 3D Routing]
```

The layer assignment uses a BFS-based approach to find optimal layer combinations that minimize via count while respecting per-layer capacity constraints.

## 8. Post-Processing

The post-processing stage performs final refinement:

1. **Overflow Reduction:** Re-routes nets that contribute to remaining overflow.
2. **Wirelength Optimization:** Adjusts paths to minimize total wirelength without increasing congestion.
3. **Via Minimization:** Reduces unnecessary layer transitions.

## References

- Yen-Jung Chang, Yu-Ting Lee, and Ting-Chi Wang, "NTHU-Route 2.0: A Fast and Stable Global Router," ICCAD 2008.
- Jhih-Rong Gao, Pei-Ci Wu, and Ting-Chi Wang, "A New Global Router for Modern Designs," ASP-DAC 2008.
- Tsung-Hsien Lee and Ting-Chi Wang, "Congestion-Constrained Layer Assignment for Via Minimization in Global Routing," IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems, 2008.
- T.-H. Lee and T.-C. Wang, "Robust Layer Assignment for Via Optimization in Multi-layer Global Routing," ISPD 2009.
