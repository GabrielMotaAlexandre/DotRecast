using System.Numerics;

namespace DotRecast.Detour.Crowd
{
    public readonly struct DtObstacleSegment
    {
        /** End points of the obstacle segment */
        public readonly Vector2 p;

        /** End points of the obstacle segment */
        public readonly Vector2 q;

        public DtObstacleSegment(Vector2 p, Vector2 q)
        {
            this.p = p;
            this.q = q;
        }
    }
}