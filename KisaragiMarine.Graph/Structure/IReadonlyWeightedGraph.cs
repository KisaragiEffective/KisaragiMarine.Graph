using System.Collections.Generic;

namespace KisaragiMarine.Graph.Structure;

public interface IReadonlyWeightedGraph<TVertex, TEdge> : IReadonlyGraph<TVertex> where TVertex: notnull
    where TEdge: notnull
{
    public IEnumerable<TEdge> EdgesFrom(TVertex vertex);

    public IEnumerable<TEdge> AllEdges();

    public bool TryGetEdge(TVertex from, TVertex to, out TEdge edge);
    
    public TEdge GetEdge(TVertex from, TVertex to);
}
