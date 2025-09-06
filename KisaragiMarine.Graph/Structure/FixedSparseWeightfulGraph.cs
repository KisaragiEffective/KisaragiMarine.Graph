using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;

namespace KisaragiMarine.Graph.Structure;

public class FixedSparseWeightfulGraph<TVertex, TEdge>(ReadOnlyMemory<TVertex> vertexIds) 
    : 
        IReadonlyDirectedGraph<TVertex>, 
        IReadonlyWeightedGraph<TVertex, TEdge> 
    where TVertex : IEquatable<TVertex>
    where TEdge: notnull

{
    private readonly Dictionary<TVertex, Dictionary<TVertex, TEdge>> _adjacency = new();
    
    public bool Contains(TVertex vertex) => vertexIds.Span.Contains(vertex);

    public IEnumerable<TVertex> OutgoingVertices(TVertex vertex) => _adjacency.TryGetValue(vertex, out var neighbors) ? neighbors.Keys : Enumerable.Empty<TVertex>();

    public IEnumerable<TEdge> EdgesFrom(TVertex vertex)
    {
        return _adjacency.TryGetValue(vertex, out var dict) ? dict.Values : Enumerable.Empty<TEdge>();
    }

    public IEnumerable<TVertex> AllVertices() => vertexIds.Span.ToImmutableArray();

    public IEnumerable<TEdge> AllEdges() => _adjacency.Values.SelectMany(v => v.Values);
    public bool TryGetEdge(TVertex from, TVertex to, out TEdge edge)
    {
        if (_adjacency.TryGetValue(from, out var edges) &&
            edges.TryGetValue(to, out edge!))
        {
            return true;
        }

        edge = default!;
        return false;
    }

    public TEdge GetEdge(TVertex from, TVertex to) => _adjacency[from][to];

    public void AddEdge(TVertex from, TVertex to, TEdge edge)
    {
        _adjacency.TryAdd(from, new Dictionary<TVertex, TEdge>());
        _adjacency[from][to] = edge;
    }
}