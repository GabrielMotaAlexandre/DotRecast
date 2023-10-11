namespace DotRecast.Recast
{
    public class RcBuilderResult
    {
        public readonly int tileX;
        public readonly int tileZ;

        public readonly RcCompactHeightfield CompactHeightfield;
        public readonly RcContourSet ContourSet;
        public readonly RcPolyMesh Mesh;
        public readonly RcPolyMeshDetail MeshDetail;
        public readonly RcHeightfield Solid;

        public RcBuilderResult(int tileX, int tileZ, in RcHeightfield solid, RcCompactHeightfield chf, RcContourSet cs, RcPolyMesh pmesh, RcPolyMeshDetail dmesh)
        {
            this.tileX = tileX;
            this.tileZ = tileZ;
            Solid = solid;
            CompactHeightfield = chf;
            ContourSet = cs;
            Mesh = pmesh;
            MeshDetail = dmesh;
        }
    }
}