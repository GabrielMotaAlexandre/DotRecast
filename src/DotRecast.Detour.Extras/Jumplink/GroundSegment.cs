using System.Numerics;

namespace DotRecast.Detour.Extras.Jumplink
{
    public class GroundSegment
    {
        public Vector3 p = new Vector3();
        public Vector3 q = new Vector3();
        public GroundSample[] gsamples;
        public float height;
    }
}