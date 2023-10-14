using DotRecast.Core;

namespace DotRecast.Detour.Io
{
    public class DtNavMeshParamsReader
    {
        public static DtNavMeshParams Read(RcByteBuffer bb)
        {
            DtNavMeshParams option = new();
            option.orig.X = bb.GetFloat();
            bb.GetFloat();
            option.orig.Y = bb.GetFloat();
            option.tileWidth = bb.GetFloat();
            option.tileHeight = bb.GetFloat();
            option.maxTiles = bb.GetInt();
            option.maxPolys = bb.GetInt();
            return option;
        }
    }
}