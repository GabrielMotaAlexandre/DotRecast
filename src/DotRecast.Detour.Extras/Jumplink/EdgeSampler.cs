using System.Collections.Generic;
using System.Numerics;

namespace DotRecast.Detour.Extras.Jumplink
{
    public class EdgeSampler
    {
        public readonly GroundSegment start = new GroundSegment();
        public readonly List<GroundSegment> end = new List<GroundSegment>();
        public readonly Trajectory trajectory;

        public readonly Vector3 ax = new Vector3();
        public readonly Vector3 ay = new Vector3();
        public readonly Vector3 az = new Vector3();

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