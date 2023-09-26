using System.Collections.Generic;
using System.Numerics;

namespace DotRecast.Detour.Extras.Jumplink
{
    public class EdgeSampler
    {
        public readonly GroundSegment start = new();
        public readonly List<GroundSegment> end = new();
        public readonly Trajectory trajectory;

        public readonly Vector3 ax = new();
        public readonly Vector3 ay = new();
        public readonly Vector3 az = new();

        public EdgeSampler(JumpEdge edge, Trajectory trajectory)
        {
            this.trajectory = trajectory;
            ax = edge.sq - (edge.sp);
            ax.Normalize();
            az = new Vector3(ax.Z, 0, -ax.X);
            az.Normalize();
            ay = new Vector3(0, 1, 0);
        }
    }
}