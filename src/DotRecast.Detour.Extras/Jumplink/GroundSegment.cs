using System.Numerics;

namespace DotRecast.Detour.Extras.Jumplink
{
    public class GroundSegment
    {
        public Vector3 p = new();
        public Vector3 q = new();
        public GroundSample[] gsamples;
        public float height;
    }
}