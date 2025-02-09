using System;
using System.Numerics;
using DotRecast.Recast.Geom;
using DotRecast.Recast.Toolset.Builder;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcOffMeshConnectionTool : IRcToolable
    {
        public RcOffMeshConnectionTool()
        {
        }

        public string GetName()
        {
            return "Off-Mesh Links";
        }

        public static void Add(IInputGeomProvider geom, RcNavMeshBuildSettings settings, Vector3 start, Vector3 end, bool bidir)
        {
            if (null == geom)
                return;

            int area = SampleAreaModifications.SAMPLE_POLYAREA_TYPE_JUMP;
            int flags = SampleAreaModifications.SAMPLE_POLYFLAGS_JUMP;
            geom.AddOffMeshConnection(start, end, settings.agentRadius, bidir, area, flags);
        }

        public static void Remove(IInputGeomProvider geom, RcNavMeshBuildSettings settings, Vector3 p)
        {
            // Delete
            // Find nearest link end-point
            float nearestDist = float.MaxValue;
            RcOffMeshConnection nearestConnection = null;
            foreach (RcOffMeshConnection offMeshCon in geom.GetOffMeshConnections())
            {
                float d = Math.Min(Vector3.DistanceSquared(p, new Vector3(offMeshCon.verts[0], offMeshCon.verts[1], offMeshCon.verts[2])), Vector3.DistanceSquared(p, new Vector3(offMeshCon.verts[3], offMeshCon.verts[4], offMeshCon.verts[5])));
                if (d < nearestDist && Math.Sqrt(d) < settings.agentRadius)
                {
                    nearestDist = d;
                    nearestConnection = offMeshCon;
                }
            }

            if (nearestConnection != null)
            {
                geom.GetOffMeshConnections().Remove(nearestConnection);
            }
        }
    }
}