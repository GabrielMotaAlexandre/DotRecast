namespace DotRecast.Recast
{
    public class RcBuilderResult
    {
        public readonly int tileX;
        public readonly int tileZ;
        
        private readonly RcCompactHeightfield chf;
        private readonly RcContourSet cs;
        private readonly RcPolyMesh pmesh;
        private readonly RcPolyMeshDetail dmesh;
        public readonly RcHeightfield Solid;

        public RcBuilderResult(int tileX, int tileZ, in RcHeightfield solid, RcCompactHeightfield chf, RcContourSet cs, RcPolyMesh pmesh, RcPolyMeshDetail dmesh)
        {
            this.tileX = tileX;
            this.tileZ = tileZ;
            this.Solid = solid;
            this.chf = chf;
            this.cs = cs;
            this.pmesh = pmesh;
            this.dmesh = dmesh;
        }

        public RcPolyMesh GetMesh()
        {
            return pmesh;
        }

        public RcPolyMeshDetail GetMeshDetail()
        {
            return dmesh;
        }

        public RcCompactHeightfield GetCompactHeightfield()
        {
            return chf;
        }

        public RcContourSet GetContourSet()
        {
            return cs;
        }
    }
}