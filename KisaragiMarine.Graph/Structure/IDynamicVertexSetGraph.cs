namespace KisaragiMarine.Graph.Structure;

public interface IDynamicVertexSetGraph<TVertex> : IGraph<TVertex>
    where TVertex: notnull
{
    public void AddVertex(TVertex vertex);
}
