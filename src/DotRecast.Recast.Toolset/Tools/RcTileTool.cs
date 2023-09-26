using System.Linq;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Recast.Geom;
using DotRecast.Recast.Toolset.Builder;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcTileTool : IRcToolable
    {
        public string GetName()
        {
            return "Tiles";
        }

        public static void RemoveAllTiles(IInputGeomProvider geom, RcNavMeshBuildSettings settings, DtNavMesh navMesh)
        {
            if (null == settings || null == geom || navMesh == null)
                return;

            var bmin = geom.GetMeshBoundsMin();
            var bmax = geom.GetMeshBoundsMax();
            RcCommons.CalcGridSize(bmin, bmax, settings.cellSize, out int gw, out int gh);

            int ts = settings.tileSize;
            int tw = (gw + ts - 1) / ts;
            int th = (gh + ts - 1) / ts;

            for (int y = 0; y < th; ++y)
            {
                for (int x = 0; x < tw; ++x)
                {
                    var tileRef = navMesh.GetTileRefAt(x, y, 0);
                    navMesh.RemoveTile(tileRef);
                }
            }
        }

        public static void BuildAllTiles(IInputGeomProvider geom, RcNavMeshBuildSettings settings, DtNavMesh navMesh)
        {
            if (null == settings || null == geom || navMesh == null)
                return;

            var bmin = geom.GetMeshBoundsMin();
            var bmax = geom.GetMeshBoundsMax();
            RcCommons.CalcGridSize(bmin, bmax, settings.cellSize, out int gw, out int gh);

            int ts = settings.tileSize;
            int tw = (gw + ts - 1) / ts;
            int th = (gh + ts - 1) / ts;

            for (int y = 0; y < th; ++y)
            {
                for (int x = 0; x < tw; ++x)
                {
                    BuildTile(geom, settings, navMesh, x, y, out _, out _, out _);
                }
            }
        }

        public static bool BuildTile(IInputGeomProvider geom, RcNavMeshBuildSettings settings, DtNavMesh navMesh, int tx, int ty, out long tileBuildTicks, out int tileTriCount, out int tileMemUsage)
        {
            tileBuildTicks = 0;
            tileTriCount = 0; // ...
            tileMemUsage = 0; // ...

            var availableTileCount = navMesh.GetAvailableTileCount();
            if (0 >= availableTileCount)
            {
                return false;
            }

            Vector3 bmin = geom.GetMeshBoundsMin();
            Vector3 bmax = geom.GetMeshBoundsMax();

            RcConfig cfg = new(
                true, settings.tileSize, settings.tileSize,
                RcConfig.CalcBorder(settings.agentRadius, settings.cellSize),
                RcPartitionType.OfValue(settings.partitioning),
                settings.cellSize, settings.cellHeight,
                settings.agentMaxSlope, settings.agentHeight, settings.agentRadius, settings.agentMaxClimb,
                settings.minRegionSize * settings.minRegionSize * settings.cellSize * settings.cellSize,
                settings.mergedRegionSize * settings.mergedRegionSize * settings.cellSize * settings.cellSize,
                settings.edgeMaxLen, settings.edgeMaxError,
                settings.vertsPerPoly,
                settings.detailSampleDist, settings.detailSampleMaxError,
                settings.filterLowHangingObstacles, settings.filterLedgeSpans, settings.filterWalkableLowHeightSpans,
                SampleAreaModifications.SAMPLE_AREAMOD_WALKABLE, true
            );

            var beginTick = RcFrequency.Ticks;
            var rb = new RcBuilder();
            var result = rb.BuildTile(geom, cfg, bmin, bmax, tx, ty, new RcAtomicInteger(0), 1);

            _ = new TileNavMeshBuilder();
            var meshData = TileNavMeshBuilder.BuildMeshData(geom, settings.cellSize, settings.cellHeight, settings.agentHeight, settings.agentRadius, settings.agentMaxClimb, RcImmutableArray.Create(result)
            ).FirstOrDefault();

            if (null == meshData)
                return false;

            navMesh.UpdateTile(meshData, 0);

            tileBuildTicks = RcFrequency.Ticks - beginTick;
            tileTriCount = 0; // ...
            tileMemUsage = 0; // ...

            return true;
        }

        public static bool BuildTile(IInputGeomProvider geom, RcNavMeshBuildSettings settings, DtNavMesh navMesh, Vector3 pos, out long tileBuildTicks, out int tileTriCount, out int tileMemUsage)
        {
            tileBuildTicks = 0;
            tileTriCount = 0;
            tileMemUsage = 0;

            if (null == settings || null == geom || navMesh == null)
                return false;

            float ts = settings.tileSize * settings.cellSize;

            Vector3 bmin = geom.GetMeshBoundsMin();

            _ = geom.GetMeshBoundsMax();

            int tx = (int)((pos.X - bmin[0]) / ts);
            int ty = (int)((pos.Z - bmin[2]) / ts);

            return BuildTile(geom, settings, navMesh, tx, ty, out tileBuildTicks, out tileTriCount, out tileMemUsage);
        }

        public static bool RemoveTile(IInputGeomProvider geom, RcNavMeshBuildSettings settings, DtNavMesh navMesh, Vector3 pos)
        {
            if (null == settings || null == geom || navMesh == null)
                return false;

            float ts = settings.tileSize * settings.cellSize;

            var bmin = geom.GetMeshBoundsMin();

            int tx = (int)((pos.X - bmin[0]) / ts);
            int ty = (int)((pos.Z - bmin[2]) / ts);

            var tileRef = navMesh.GetTileRefAt(tx, ty, 0);
            navMesh.RemoveTile(tileRef);

            return true;
        }
    }
}