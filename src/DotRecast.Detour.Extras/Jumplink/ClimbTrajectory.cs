using System;
using System.Numerics;

namespace DotRecast.Detour.Extras.Jumplink
{
    public class ClimbTrajectory : Trajectory
    {
        public override Vector3 Apply(Vector3 start, Vector3 end, float u)
        {
            return new Vector3()
            {
                X = Lerp(start.X, end.X, Math.Min(2f * u, 1f)),
                Y = Lerp(start.Y, end.Y, Math.Max(0f, 2f * u - 1f)),
                Z = Lerp(start.Z, end.Z, Math.Min(2f * u, 1f))
            };
        }
    }
}