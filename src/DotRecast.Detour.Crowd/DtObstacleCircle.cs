using System.Numerics;

namespace DotRecast.Detour.Crowd
{
    /// < Max number of adaptive rings.
    public class DtObstacleCircle
    {
        /** Position of the obstacle */
        public Vector2 p;

        /** Velocity of the obstacle */
        public Vector2 vel;

        /** Velocity of the obstacle */
        public Vector2 dvel;

        /** Radius of the obstacle */
        public float rad;

        /** Use for side selection during sampling. */
        public Vector2 dp;

        /** Use for side selection during sampling. */
        public Vector2 np;
    }
}