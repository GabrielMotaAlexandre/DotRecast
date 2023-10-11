using System.Numerics;
using DotRecast.Core;

using DotRecast.Recast;

namespace DotRecast.Detour.Extras.Jumplink
{
    class NavMeshGroundSampler : AbstractGroundSampler
    {
        public override void Sample(JumpLinkBuilderConfig acfg, RcBuilderResult result, EdgeSampler es)
        {
            DtNavMeshQuery navMeshQuery = CreateNavMesh(result, acfg.agentRadius, acfg.agentHeight, acfg.agentClimb);
            SampleGround(acfg, es, (Vector3 pt, float heightRange, out float height) => GetNavMeshHeight(navMeshQuery, pt, acfg.cellSize, heightRange, out height));
        }

        private static DtNavMeshQuery CreateNavMesh(RcBuilderResult r, float agentRadius, float agentHeight, float agentClimb)
        {
            DtNavMeshCreateParams option = new()
            {
                verts = r.Mesh.verts,
                vertCount = r.Mesh.nverts,
                polys = r.Mesh.polys,
                polyAreas = r.Mesh.areas,
                polyFlags = r.Mesh.flags,
                polyCount = r.Mesh.npolys,
                nvp = r.Mesh.nvp,
                detailMeshes = r.MeshDetail.meshes,
                detailVerts = r.MeshDetail.verts,
                detailVertsCount = r.MeshDetail.nverts,
                detailTris = r.MeshDetail.tris,
                detailTriCount = r.MeshDetail.ntris,
                walkableRadius = agentRadius,
                walkableHeight = agentHeight,
                walkableClimb = agentClimb,
                bmin = r.Mesh.bmin,
                bmax = r.Mesh.bmax,
                cs = r.Mesh.cs,
                ch = r.Mesh.ch,
                buildBvTree = true
            };
            return new DtNavMeshQuery(new DtNavMesh(DtNavMeshBuilder.CreateNavMeshData(option), option.nvp, 0));
        }


        private static bool GetNavMeshHeight(DtNavMeshQuery navMeshQuery, Vector3 pt, float cs, float heightRange, out float height)
        {
            height = default;

            Vector3 halfExtents = new()
            { X = cs, Y = heightRange, Z = cs };
            float maxHeight = pt.Y + heightRange;
            RcAtomicBoolean found = new();
            RcAtomicFloat minHeight = new(pt.Y);

            navMeshQuery.QueryPolygons(pt, halfExtents, DtQueryNoOpFilter.Shared, new PolyQueryInvoker((tile, poly, refs) =>
            {
                var status = navMeshQuery.GetPolyHeight(refs, pt, out var h);
                if (status.Succeeded())
                {
                    if (h > minHeight.Get() && h < maxHeight)
                    {
                        minHeight.Exchange(h);
                        found.Set(true);
                    }
                }
            }));

            if (found.Get())
            {
                height = minHeight.Get();
                return true;
            }

            height = pt.Y;
            return false;
        }
    }
}