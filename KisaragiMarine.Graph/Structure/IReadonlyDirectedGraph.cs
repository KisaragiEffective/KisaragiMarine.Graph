using System.Collections.Generic;

namespace KisaragiMarine.Graph.Structure;

public interface IReadonlyDirectedGraph<TVertex> : IReadonlyGraph<TVertex> where TVertex : notnull
{
    public IEnumerable<TVertex> OutgoingVertices(TVertex from);
}
