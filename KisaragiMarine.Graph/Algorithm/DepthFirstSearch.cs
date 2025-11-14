using System;
using System.Collections.Generic;
using KisaragiMarine.Graph.Structure;

namespace KisaragiMarine.Graph.Algorithm;

/// <summary>
/// Depth-First Search (DFS) algorithm for graph traversal.
/// </summary>
public class DepthFirstSearch
{
    /// <summary>
    /// Performs DFS traversal on the entire graph.
    /// </summary>
    /// <param name="graph">The directed graph to traverse.</param>
    /// <returns>DFS result containing discovery and finish times.</returns>
    public DfsResult<T> Compute<T>(IReadonlyDirectedGraph<T> graph) where T : notnull
    {
        var discoveryTime = new Dictionary<T, int>();
        var finishTime = new Dictionary<T, int>();
        var parents = new Dictionary<T, T?>();
        var visitOrder = new List<T>();
        var time = 0;

        void Visit(T vertex)
        {
            time++;
            discoveryTime[vertex] = time;
            visitOrder.Add(vertex);

            foreach (var neighbor in graph.OutgoingVertices(vertex))
            {
                if (!discoveryTime.ContainsKey(neighbor))
                {
                    parents[neighbor] = vertex;
                    Visit(neighbor);
                }
            }

            time++;
            finishTime[vertex] = time;
        }

        foreach (var vertex in graph.AllVertices())
        {
            if (!discoveryTime.ContainsKey(vertex))
            {
                parents[vertex] = default;
                Visit(vertex);
            }
        }

        return new DfsResult<T>(discoveryTime, finishTime, parents, visitOrder);
    }

    /// <summary>
    /// Performs DFS traversal starting from the specified vertex.
    /// </summary>
    /// <param name="graph">The directed graph to traverse.</param>
    /// <param name="start">The starting vertex.</param>
    /// <returns>DFS result containing discovery and finish times.</returns>
    public DfsResult<T> ComputeFrom<T>(IReadonlyDirectedGraph<T> graph, T start) where T : notnull
    {
        if (!graph.Contains(start))
        {
            throw new ArgumentException("Start vertex is not in the graph", nameof(start));
        }

        var discoveryTime = new Dictionary<T, int>();
        var finishTime = new Dictionary<T, int>();
        var parents = new Dictionary<T, T?>();
        var visitOrder = new List<T>();
        var time = 0;

        void Visit(T vertex)
        {
            time++;
            discoveryTime[vertex] = time;
            visitOrder.Add(vertex);

            foreach (var neighbor in graph.OutgoingVertices(vertex))
            {
                if (!discoveryTime.ContainsKey(neighbor))
                {
                    parents[neighbor] = vertex;
                    Visit(neighbor);
                }
            }

            time++;
            finishTime[vertex] = time;
        }

        parents[start] = default;
        Visit(start);

        return new DfsResult<T>(discoveryTime, finishTime, parents, visitOrder);
    }
}

/// <summary>
/// Result of DFS traversal containing timing and parent information.
/// </summary>
public class DfsResult<T> where T : notnull
{
    private readonly Dictionary<T, int> _discoveryTime;
    private readonly Dictionary<T, int> _finishTime;
    private readonly Dictionary<T, T?> _parents;
    private readonly List<T> _visitOrder;

    internal DfsResult(
        Dictionary<T, int> discoveryTime,
        Dictionary<T, int> finishTime,
        Dictionary<T, T?> parents,
        List<T> visitOrder)
    {
        _discoveryTime = discoveryTime;
        _finishTime = finishTime;
        _parents = parents;
        _visitOrder = visitOrder;
    }

    /// <summary>
    /// Gets the discovery time of the specified vertex.
    /// Returns -1 if the vertex was not visited.
    /// </summary>
    public int GetDiscoveryTime(T vertex)
    {
        return _discoveryTime.TryGetValue(vertex, out var time) ? time : -1;
    }

    /// <summary>
    /// Gets the finish time of the specified vertex.
    /// Returns -1 if the vertex was not visited.
    /// </summary>
    public int GetFinishTime(T vertex)
    {
        return _finishTime.TryGetValue(vertex, out var time) ? time : -1;
    }

    /// <summary>
    /// Gets the parent of the specified vertex in the DFS tree.
    /// Returns null if the vertex is a root or was not visited.
    /// </summary>
    public T? GetParent(T vertex)
    {
        return _parents.TryGetValue(vertex, out var parent) ? parent : default;
    }

    /// <summary>
    /// Checks if the specified vertex was visited during DFS.
    /// </summary>
    public bool WasVisited(T vertex)
    {
        return _discoveryTime.ContainsKey(vertex);
    }

    /// <summary>
    /// Gets the vertices in the order they were discovered.
    /// </summary>
    public IReadOnlyList<T> VisitOrder => _visitOrder;

    /// <summary>
    /// Gets all visited vertices.
    /// </summary>
    public IEnumerable<T> VisitedVertices => _discoveryTime.Keys;
}

public static class DepthFirstSearchExtensions
{
    /// <summary>
    /// Performs DFS traversal on the entire graph.
    /// </summary>
    public static DfsResult<T> DepthFirstSearch<T>(this IReadonlyDirectedGraph<T> self)
        where T : notnull
    {
        return new DepthFirstSearch().Compute(self);
    }

    /// <summary>
    /// Performs DFS traversal starting from the specified vertex.
    /// </summary>
    public static DfsResult<T> DepthFirstSearchFrom<T>(this IReadonlyDirectedGraph<T> self, T start)
        where T : notnull
    {
        return new DepthFirstSearch().ComputeFrom(self, start);
    }
}
