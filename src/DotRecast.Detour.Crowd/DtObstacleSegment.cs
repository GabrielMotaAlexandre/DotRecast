using System.Numerics;

namespace DotRecast.Detour.Crowd
{
    public class DtObstacleSegment
    {
        /** End points of the obstacle segment */
        public Vector3 p = new();

        /** End points of the obstacle segment */
        public Vector3 q = new();

        public bool touch;
    }
}