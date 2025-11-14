using System;
using System.Collections.Generic;
using System.Numerics;
using KisaragiMarine.Graph.Structure;

namespace KisaragiMarine.Graph.Algorithm;

/// <summary>
/// Dijkstra's shortest path algorithm for weighted graphs with non-negative weights.
/// </summary>
public class Dijkstra
{
    /// <summary>
    /// Computes shortest paths from a start vertex to all other vertices using Dijkstra's algorithm.
    /// Requires non-negative edge weights.
    /// </summary>
    /// <param name="graph">The weighted directed graph.</param>
    /// <param name="start">The starting vertex.</param>
    /// <param name="getWeight">Function to extract weight from edge.</param>
    /// <returns>Result containing shortest distances and paths.</returns>
    public DijkstraResult<TVertex, TWeight> Compute<TVertex, TEdge, TWeight>(
        IReadonlyDirectedGraph<TVertex> graph,
        TVertex start,
        Func<TVertex, TVertex, TEdge> getEdge,
        Func<TEdge, TWeight> getWeight)
        where TVertex : notnull
        where TEdge : notnull
        where TWeight : INumber<TWeight>, IMinMaxValue<TWeight>
    {
        if (!graph.Contains(start))
        {
            throw new ArgumentException("Start vertex is not in the graph", nameof(start));
        }

        var distances = new Dictionary<TVertex, TWeight>();
        var parents = new Dictionary<TVertex, TVertex?>();
        var visited = new HashSet<TVertex>();
        var priorityQueue = new PriorityQueue<TVertex, TWeight>();

        // Initialize distances
        foreach (var vertex in graph.AllVertices())
        {
            distances[vertex] = TWeight.MaxValue;
        }
        distances[start] = TWeight.Zero;
        parents[start] = default;

        priorityQueue.Enqueue(start, TWeight.Zero);

        while (priorityQueue.Count > 0)
        {
            var current = priorityQueue.Dequeue();

            if (visited.Contains(current))
            {
                continue;
            }

            visited.Add(current);

            foreach (var neighbor in graph.OutgoingVertices(current))
            {
                if (visited.Contains(neighbor))
                {
                    continue;
                }

                var edge = getEdge(current, neighbor);
                var weight = getWeight(edge);

                if (weight < TWeight.Zero)
                {
                    throw new ArgumentException("Dijkstra's algorithm requires non-negative edge weights");
                }

                var newDistance = distances[current] + weight;

                if (newDistance < distances[neighbor])
                {
                    distances[neighbor] = newDistance;
                    parents[neighbor] = current;
                    priorityQueue.Enqueue(neighbor, newDistance);
                }
            }
        }

        return new DijkstraResult<TVertex, TWeight>(distances, parents, start);
    }
}

/// <summary>
/// Result of Dijkstra's shortest path algorithm.
/// </summary>
public class DijkstraResult<TVertex, TWeight>
    where TVertex : notnull
    where TWeight : INumber<TWeight>, IMinMaxValue<TWeight>
{
    private readonly Dictionary<TVertex, TWeight> _distances;
    private readonly Dictionary<TVertex, TVertex?> _parents;
    private readonly TVertex _start;

    internal DijkstraResult(Dictionary<TVertex, TWeight> distances, Dictionary<TVertex, TVertex?> parents, TVertex start)
    {
        _distances = distances;
        _parents = parents;
        _start = start;
    }

    /// <summary>
    /// Gets the shortest distance from the start vertex to the specified vertex.
    /// Returns TWeight.MaxValue if the vertex is not reachable.
    /// </summary>
    public TWeight GetDistance(TVertex vertex)
    {
        return _distances.TryGetValue(vertex, out var distance) ? distance : TWeight.MaxValue;
    }

    /// <summary>
    /// Gets the parent of the specified vertex in the shortest path tree.
    /// Returns null if the vertex is the start vertex or not reachable.
    /// </summary>
    public TVertex? GetParent(TVertex vertex)
    {
        return _parents.TryGetValue(vertex, out var parent) ? parent : default;
    }

    /// <summary>
    /// Checks if the specified vertex is reachable from the start vertex.
    /// </summary>
    public bool IsReachable(TVertex vertex)
    {
        return _distances.TryGetValue(vertex, out var distance) && distance < TWeight.MaxValue;
    }

    /// <summary>
    /// Reconstructs the shortest path from the start vertex to the target vertex.
    /// Returns null if the target is not reachable.
    /// </summary>
    public IReadOnlyList<TVertex>? GetPath(TVertex target)
    {
        if (!IsReachable(target))
        {
            return null;
        }

        var path = new List<TVertex>();
        var current = (TVertex?)target;

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
    public IEnumerable<TVertex> ReachableVertices
    {
        get
        {
            foreach (var kvp in _distances)
            {
                if (kvp.Value < TWeight.MaxValue)
                {
                    yield return kvp.Key;
                }
            }
        }
    }
}

public static class DijkstraExtensions
{
    /// <summary>
    /// Computes shortest paths using Dijkstra's algorithm for a weighted graph.
    /// </summary>
    public static DijkstraResult<TVertex, TWeight> Dijkstra<TVertex, TEdge, TWeight>(
        this IReadonlyWeightedGraph<TVertex, TEdge> self,
        TVertex start,
        Func<TEdge, TWeight> getWeight)
        where TVertex : notnull
        where TEdge : notnull
        where TWeight : INumber<TWeight>, IMinMaxValue<TWeight>
    {
        return new Dijkstra().Compute<TVertex, TEdge, TWeight>(
            self,
            start,
            (from, to) => self.GetEdge(from, to),
            getWeight);
    }

    /// <summary>
    /// Computes shortest paths using Dijkstra's algorithm when edge type is the weight itself.
    /// </summary>
    public static DijkstraResult<TVertex, TWeight> Dijkstra<TVertex, TWeight>(
        this IReadonlyWeightedGraph<TVertex, TWeight> self,
        TVertex start)
        where TVertex : notnull
        where TWeight : INumber<TWeight>, IMinMaxValue<TWeight>
    {
        return new Dijkstra().Compute<TVertex, TWeight, TWeight>(
            self,
            start,
            (from, to) => self.GetEdge(from, to),
            weight => weight);
    }
}
