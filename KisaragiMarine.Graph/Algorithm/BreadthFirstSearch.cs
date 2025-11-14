using System;
using System.Collections.Generic;
using KisaragiMarine.Graph.Structure;

namespace KisaragiMarine.Graph.Algorithm;

/// <summary>
/// Breadth-First Search (BFS) algorithm for graph traversal.
/// </summary>
public class BreadthFirstSearch
{
    /// <summary>
    /// Performs BFS traversal starting from the specified vertex.
    /// </summary>
    /// <param name="graph">The directed graph to traverse.</param>
    /// <param name="start">The starting vertex.</param>
    /// <returns>BFS result containing distances and parent information.</returns>
    public BfsResult<T> Compute<T>(IReadonlyDirectedGraph<T> graph, T start) where T : notnull
    {
        if (!graph.Contains(start))
        {
            throw new ArgumentException("Start vertex is not in the graph", nameof(start));
        }

        var distances = new Dictionary<T, int>();
        var parents = new Dictionary<T, T?>();
        var visited = new HashSet<T>();
        var queue = new Queue<T>();

        distances[start] = 0;
        parents[start] = default;
        visited.Add(start);
        queue.Enqueue(start);

        while (queue.Count > 0)
        {
            var current = queue.Dequeue();

            foreach (var neighbor in graph.OutgoingVertices(current))
            {
                if (!visited.Add(neighbor))
                {
                    continue;
                }

                distances[neighbor] = distances[current] + 1;
                parents[neighbor] = current;
                queue.Enqueue(neighbor);
            }
        }

        return new BfsResult<T>(distances, parents, start);
    }
}

/// <summary>
/// Result of BFS traversal containing distances and parent information.
/// </summary>
public class BfsResult<T> where T : notnull
{
    private readonly Dictionary<T, int> _distances;
    private readonly Dictionary<T, T?> _parents;
    private readonly T _start;

    internal BfsResult(Dictionary<T, int> distances, Dictionary<T, T?> parents, T start)
    {
        _distances = distances;
        _parents = parents;
        _start = start;
    }

    /// <summary>
    /// Gets the distance from the start vertex to the specified vertex.
    /// Returns -1 if the vertex is not reachable.
    /// </summary>
    public int GetDistance(T vertex)
    {
        return _distances.TryGetValue(vertex, out var distance) ? distance : -1;
    }

    /// <summary>
    /// Gets the parent of the specified vertex in the BFS tree.
    /// Returns null if the vertex is the start vertex or not reachable.
    /// </summary>
    public T? GetParent(T vertex)
    {
        return _parents.TryGetValue(vertex, out var parent) ? parent : default;
    }

    /// <summary>
    /// Checks if the specified vertex is reachable from the start vertex.
    /// </summary>
    public bool IsReachable(T vertex)
    {
        return _distances.ContainsKey(vertex);
    }

    /// <summary>
    /// Reconstructs the shortest path from the start vertex to the target vertex.
    /// Returns null if the target is not reachable.
    /// </summary>
    public IReadOnlyList<T>? GetPath(T target)
    {
        if (!IsReachable(target))
        {
            return null;
        }

        var path = new List<T>();
        var current = target;

        while (current != null && !current.Equals(_start))
        {
            path.Add(current);
            current = GetParent(current);
        }

        path.Add(_start);
        path.Reverse();

        return path;
    }

    /// <summary>
    /// Gets all vertices reachable from the start vertex.
    /// </summary>
    public IEnumerable<T> ReachableVertices => _distances.Keys;
}

public static class BreadthFirstSearchExtensions
{
    /// <summary>
    /// Performs BFS traversal starting from the specified vertex.
    /// </summary>
    public static BfsResult<T> BreadthFirstSearch<T>(this IReadonlyDirectedGraph<T> self, T start)
        where T : notnull
    {
        return new BreadthFirstSearch().Compute(self, start);
    }
}
