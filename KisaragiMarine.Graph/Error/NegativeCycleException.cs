using System;

namespace KisaragiMarine.Graph.Error;

/// <summary>
/// Exception thrown when a negative cycle is detected in the graph.
/// </summary>
public class NegativeCycleException : Exception
{
    public NegativeCycleException() : base("Graph contains a negative cycle")
    {
    }

    public NegativeCycleException(string message) : base(message)
    {
    }

    public NegativeCycleException(string message, Exception innerException) : base(message, innerException)
    {
    }
}
