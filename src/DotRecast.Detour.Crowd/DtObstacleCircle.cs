using System.Numerics;

namespace DotRecast.Detour.Crowd
{
    /// < Max number of adaptive rings.
    public class DtObstacleCircle
    {
        /** Position of the obstacle */
        public Vector3 p = new();

        /** Velocity of the obstacle */
        public Vector2 vel = new();

        /** Velocity of the obstacle */
        public Vector2 dvel = new();

        /** Radius of the obstacle */
        public float rad;

        /** Use for side selection during sampling. */
        public Vector3 dp = new();

        /** Use for side selection during sampling. */
        public Vector3 np = new();
    }
}