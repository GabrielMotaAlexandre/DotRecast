using System.Numerics;

namespace DotRecast.Detour.Crowd
{
    public struct DtSegment
    {
        public Vector2 Start;
        public Vector2 End;

        ///** Segment start/end */
        //public Vector2[] s = new Vector2[2];

        /** Distance for pruning. */
        public float PruningDistance;
    }
}