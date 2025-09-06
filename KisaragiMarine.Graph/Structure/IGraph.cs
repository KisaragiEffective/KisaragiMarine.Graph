namespace KisaragiMarine.Graph.Structure;

public interface IGraph<TVertex> : IReadonlyGraph<TVertex>
    where TVertex: notnull
{
    public void AddEdge(TVertex from, TVertex to);
}
