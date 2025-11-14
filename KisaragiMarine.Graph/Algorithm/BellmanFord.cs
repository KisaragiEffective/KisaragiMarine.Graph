using System;
using System.Collections.Generic;
using System.Numerics;
using KisaragiMarine.Graph.Error;
using KisaragiMarine.Graph.Structure;

namespace KisaragiMarine.Graph.Algorithm;

/// <summary>
/// Bellman-Ford shortest path algorithm for weighted graphs.
/// Supports negative edge weights and detects negative cycles.
/// </summary>
public class BellmanFord
{
    /// <summary>
    /// Computes shortest paths from a start vertex to all other vertices using Bellman-Ford algorithm.
    /// Supports negative edge weights and detects negative cycles.
    /// </summary>
    /// <param name="graph">The weighted directed graph.</param>
    /// <param name="start">The starting vertex.</param>
    /// <param name="getWeight">Function to extract weight from edge.</param>
    /// <returns>Result containing shortest distances and paths.</returns>
    /// <exception cref="NegativeCycleException">Thrown when a negative cycle is detected.</exception>
    public BellmanFordResult<TVertex, TWeight> Compute<TVertex, TEdge, TWeight>(
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
        var vertices = new List<TVertex>();

        // Initialize distances
        foreach (var vertex in graph.AllVertices())
        {
            distances[vertex] = TWeight.MaxValue;
            vertices.Add(vertex);
        }
        distances[start] = TWeight.Zero;
        parents[start] = default;

        var vertexCount = vertices.Count;

        // Relax edges |V| - 1 times
        for (int i = 0; i < vertexCount - 1; i++)
        {
            bool updated = false;

            foreach (var u in vertices)
            {
                if (distances[u] == TWeight.MaxValue)
                {
                    continue;
                }

                foreach (var v in graph.OutgoingVertices(u))
                {
                    var edge = getEdge(u, v);
                    var weight = getWeight(edge);
                    var newDistance = distances[u] + weight;

                    if (newDistance < distances[v])
                    {
                        distances[v] = newDistance;
                        parents[v] = u;
                        updated = true;
                    }
                }
            }

            // Early termination if no updates occurred
            if (!updated)
            {
                break;
            }
        }

        // Check for negative cycles
        foreach (var u in vertices)
        {
            if (distances[u] == TWeight.MaxValue)
            {
                continue;
            }

            foreach (var v in graph.OutgoingVertices(u))
            {
                var edge = getEdge(u, v);
                var weight = getWeight(edge);
                var newDistance = distances[u] + weight;

                if (newDistance < distances[v])
                {
                    throw new NegativeCycleException();
                }
            }
        }

        return new BellmanFordResult<TVertex, TWeight>(distances, parents, start);
    }
}

/// <summary>
/// Result of Bellman-Ford shortest path algorithm.
/// </summary>
public class BellmanFordResult<TVertex, TWeight>
    where TVertex : notnull
    where TWeight : INumber<TWeight>, IMinMaxValue<TWeight>
{
    private readonly Dictionary<TVertex, TWeight> _distances;
    private readonly Dictionary<TVertex, TVertex?> _parents;
    private readonly TVertex _start;

    internal BellmanFordResult(Dictionary<TVertex, TWeight> distances, Dictionary<TVertex, TVertex?> parents, TVertex start)
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

public static class BellmanFordExtensions
{
    /// <summary>
    /// Computes shortest paths using Bellman-Ford algorithm for a weighted graph.
    /// Supports negative edge weights and detects negative cycles.
    /// </summary>
    public static BellmanFordResult<TVertex, TWeight> BellmanFord<TVertex, TEdge, TWeight>(
        this IReadonlyWeightedGraph<TVertex, TEdge> self,
        TVertex start,
        Func<TEdge, TWeight> getWeight)
        where TVertex : notnull
        where TEdge : notnull
        where TWeight : INumber<TWeight>, IMinMaxValue<TWeight>
    {
        return new BellmanFord().Compute<TVertex, TEdge, TWeight>(
            self,
            start,
            (from, to) => self.GetEdge(from, to),
            getWeight);
    }

    /// <summary>
    /// Computes shortest paths using Bellman-Ford algorithm when edge type is the weight itself.
    /// Supports negative edge weights and detects negative cycles.
    /// </summary>
    public static BellmanFordResult<TVertex, TWeight> BellmanFord<TVertex, TWeight>(
        this IReadonlyWeightedGraph<TVertex, TWeight> self,
        TVertex start)
        where TVertex : notnull
        where TWeight : INumber<TWeight>, IMinMaxValue<TWeight>
    {
        return new BellmanFord().Compute<TVertex, TWeight, TWeight>(
            self,
            start,
            (from, to) => self.GetEdge(from, to),
            weight => weight);
    }
}
