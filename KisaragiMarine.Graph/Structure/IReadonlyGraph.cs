using System.Collections.Generic;
using System.Linq;

namespace KisaragiMarine.Graph.Structure;

public interface IReadonlyGraph<TVertex> where TVertex: notnull
{
    public bool Contains(TVertex vertex);

    // public IEnumerable<TVertex> AdjacentVertices(TVertex from);

    public IEnumerable<TVertex> AllVertices();

    public long CountVertices() => AllVertices().LongCount();
}
