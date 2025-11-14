using System;
using System.Collections.Generic;
using System.Linq;
using KisaragiMarine.Graph.Structure;

namespace KisaragiMarine.Graph.Algorithm;

/// <summary>
/// Tarjan's algorithm for finding strongly connected components in a directed graph.
/// </summary>
public class StronglyConnectedComponents
{
    /// <summary>
    /// Computes strongly connected components using Tarjan's algorithm.
    /// </summary>
    /// <param name="graph">The directed graph.</param>
    /// <returns>Result containing the strongly connected components.</returns>
    public SccResult<T> Compute<T>(IReadonlyDirectedGraph<T> graph) where T : notnull
    {
        var index = 0;
        var stack = new Stack<T>();
        var indices = new Dictionary<T, int>();
        var lowlinks = new Dictionary<T, int>();
        var onStack = new HashSet<T>();
        var components = new List<List<T>>();

        void StrongConnect(T v)
        {
            indices[v] = index;
            lowlinks[v] = index;
            index++;
            stack.Push(v);
            onStack.Add(v);

            foreach (var w in graph.OutgoingVertices(v))
            {
                if (!indices.ContainsKey(w))
                {
                    StrongConnect(w);
                    lowlinks[v] = Math.Min(lowlinks[v], lowlinks[w]);
                }
                else if (onStack.Contains(w))
                {
                    lowlinks[v] = Math.Min(lowlinks[v], indices[w]);
                }
            }

            // If v is a root node, pop the stack and create an SCC
            if (lowlinks[v] == indices[v])
            {
                var component = new List<T>();
                T w;
                do
                {
                    w = stack.Pop();
                    onStack.Remove(w);
                    component.Add(w);
                } while (!w.Equals(v));

                components.Add(component);
            }
        }

        foreach (var vertex in graph.AllVertices())
        {
            if (!indices.ContainsKey(vertex))
            {
                StrongConnect(vertex);
            }
        }

        return new SccResult<T>(components);
    }
}

/// <summary>
/// Result of strongly connected components computation.
/// </summary>
public class SccResult<T> where T : notnull
{
    private readonly List<List<T>> _components;
    private readonly Dictionary<T, int> _componentIds;

    internal SccResult(List<List<T>> components)
    {
        _components = components;
        _componentIds = new Dictionary<T, int>();

        for (int i = 0; i < components.Count; i++)
        {
            foreach (var vertex in components[i])
            {
                _componentIds[vertex] = i;
            }
        }
    }

    /// <summary>
    /// Gets the number of strongly connected components.
    /// </summary>
    public int ComponentCount => _components.Count;

    /// <summary>
    /// Gets all strongly connected components.
    /// </summary>
    public IReadOnlyList<IReadOnlyList<T>> Components =>
        _components.Select(c => (IReadOnlyList<T>)c).ToList();

    /// <summary>
    /// Gets the component ID for the specified vertex.
    /// Returns -1 if the vertex is not in the graph.
    /// </summary>
    public int GetComponentId(T vertex)
    {
        return _componentIds.TryGetValue(vertex, out var id) ? id : -1;
    }

    /// <summary>
    /// Gets the strongly connected component containing the specified vertex.
    /// Returns null if the vertex is not in the graph.
    /// </summary>
    public IReadOnlyList<T>? GetComponent(T vertex)
    {
        var id = GetComponentId(vertex);
        return id >= 0 ? _components[id] : null;
    }

    /// <summary>
    /// Checks if two vertices are in the same strongly connected component.
    /// </summary>
    public bool AreStronglyConnected(T vertex1, T vertex2)
    {
        var id1 = GetComponentId(vertex1);
        var id2 = GetComponentId(vertex2);
        return id1 >= 0 && id1 == id2;
    }

    /// <summary>
    /// Checks if the graph is strongly connected (i.e., has only one component).
    /// </summary>
    public bool IsStronglyConnected => _components.Count == 1;
}

public static class StronglyConnectedComponentsExtensions
{
    /// <summary>
    /// Computes strongly connected components using Tarjan's algorithm.
    /// </summary>
    public static SccResult<T> FindStronglyConnectedComponents<T>(this IReadonlyDirectedGraph<T> self)
        where T : notnull
    {
        return new StronglyConnectedComponents().Compute(self);
    }
}
