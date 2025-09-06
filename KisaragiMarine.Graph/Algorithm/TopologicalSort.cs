using System;
using System.Collections.Generic;
using KisaragiMarine.Graph.Error;
using KisaragiMarine.Graph.Structure;

namespace KisaragiMarine.Graph.Algorithm;

public class TopologicalSort
{
    public void Compute<T>(IReadonlyDirectedGraph<T> graph, Span<T> topo) where T : notnull
    {
        var vertexCount = graph.CountVertices();
        if (vertexCount > int.MaxValue)
        {
            throw new ArgumentOutOfRangeException(nameof(graph), "vertex count exceeds Int32.MaxValue");
        }
        
        if (vertexCount != topo.Length)
        {
            throw new ArgumentException("out buffer length must match with graph's vertex count", nameof(topo));
        }

        var marks = new Dictionary<T, Mark>((int) vertexCount);
        var writeIndex = (int) vertexCount - 1;

        // ReSharper disable once MoveLocalFunctionAfterJumpStatement
        void Visit(T vertexToVisit, Span<T> buffer)
        {
            if (marks.TryGetValue(vertexToVisit, out var state)) {
                if (state == Mark.PermanentlyVisited)
                {
                    return;
                }

                if (state == Mark.TemporaryVisited)
                {
                    throw new CyclicGraphException();
                }
            }

            marks[vertexToVisit] = Mark.TemporaryVisited;
            
            foreach (var outGoingVertex in graph.OutgoingVertices(vertexToVisit))
            {
                // TODO: make this non-recursive (stack-based DFS) if deep graphs are expected
                Visit(outGoingVertex, buffer);
            }

            marks[vertexToVisit] = Mark.PermanentlyVisited;
            buffer[writeIndex] = vertexToVisit;
            writeIndex -= 1;
        }

        
        foreach (var vertex in graph.AllVertices())
        {
            if (!marks.ContainsKey(vertex))
            {
                Visit(vertex, topo);
            }
        }
    }
}

public static class TopologicalSortExtensions
{
    public static T[] OrderByTopologicalSort<T>(this IReadonlyDirectedGraph<T> self) where T : notnull
    {
        var list = new T[(int)self.CountVertices()];
        new TopologicalSort().Compute(self, list);

        return list;
    }
}

file enum Mark
{
    TemporaryVisited,
    PermanentlyVisited,
}
