using System.Numerics;

namespace DotRecast.Recast.Demo.Messages;

public class RaycastEvent : IRecastDemoMessage
{
    public required Vector3 Start { get; init; }
    public required Vector3 End { get; init; }
}