namespace KisaragiMarine.Graph.Structure;

public interface IWeightedGraph<TVertex, TEdge> : IReadonlyWeightedGraph<TVertex, TEdge>
    where TVertex: notnull
    where TEdge: notnull
{
    public void AddWeightedEdge(TVertex from, TVertex to, TEdge weight);
}
