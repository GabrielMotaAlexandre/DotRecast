using System;
using System.Linq;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Detour.TileCache;
using DotRecast.Detour.TileCache.Io.Compress;
using DotRecast.Recast.Geom;
using DotRecast.Recast.Toolset.Builder;
using DotRecast.Recast.Toolset.Geom;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcObstacleTool : IRcToolable
    {
        private readonly IDtTileCacheCompressorFactory _comp;
        private readonly DemoDtTileCacheMeshProcess _proc;
        private DtTileCache _tc;

        public RcObstacleTool(IDtTileCacheCompressorFactory comp)
        {
            _comp = comp;
            _proc = new DemoDtTileCacheMeshProcess();
        }

        public string GetName()
        {
            return "Temp Obstacles";
        }

        public NavMeshBuildResult Build(IInputGeomProvider geom, RcNavMeshBuildSettings setting, RcByteOrder order, bool cCompatibility)
        {
            if (null == geom || null == geom.GetMesh())
            {
                //m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
                return new NavMeshBuildResult();
            }

            _proc.Init(geom);

            // Init cache
            var bmin = geom.GetMeshBoundsMin();
            var bmax = geom.GetMeshBoundsMax();
            RcCommons.CalcGridSize(bmin, bmax, setting.cellSize, out var gw, out var gh);
            int ts = setting.tileSize;
            int tw = (gw + ts - 1) / ts;
            int th = (gh + ts - 1) / ts;

            // Generation params.
            var walkableRadius = (int)Math.Ceiling(setting.agentRadius / setting.cellSize); // Reserve enough padding.
            RcConfig cfg = new(
                true, setting.tileSize, setting.tileSize,
                walkableRadius + 3,
                RcPartitionType.OfValue(setting.partitioning),
                setting.cellSize, setting.cellHeight,
                setting.agentMaxSlope, setting.agentHeight, setting.agentRadius, setting.agentMaxClimb,
                (int)RcMath.Sqr(setting.minRegionSize), (int)RcMath.Sqr(setting.mergedRegionSize), // Note: area = size*size
                (int)(setting.edgeMaxLen / setting.cellSize), setting.edgeMaxError,
                setting.vertsPerPoly,
                setting.detailSampleDist, setting.detailSampleMaxError,
                true, true, true,
                SampleAreaModifications.SAMPLE_AREAMOD_WALKABLE, true);

            var builder = new DtTileCacheLayerBuilder(DtTileCacheCompressorFactory.Shared);
            var storageParams = new DtTileCacheStorageParams(order, cCompatibility);
            var results = builder.Build(geom, cfg, storageParams, 8, tw, th);
            var layers = results
                .SelectMany(x => x.layers)
                .ToList();

            _tc = CreateTileCache(geom, setting, tw, th, order, cCompatibility);

            for (int i = 0; i < layers.Count; ++i)
            {
                var layer = layers[i];
                var refs = _tc.AddTile(layer, 0);
                _tc.BuildNavMeshTile(refs);
            }

            return new NavMeshBuildResult(RcImmutableArray<RcBuilderResult>.Empty, _tc.GetNavMesh());
        }

        public void ClearAllTempObstacles()
        {
            if (null == _tc)
                return;

            for (int i = 0; i < _tc.GetObstacleCount(); ++i)
            {
                DtTileCacheObstacle ob = _tc.GetObstacle(i);
                if (ob.state == DtObstacleState.DT_OBSTACLE_EMPTY)
                    continue;

                _tc.RemoveObstacle(DtTileCache.GetObstacleRef(ob));
            }
        }

        public void RemoveTempObstacle(Vector3 sp, Vector3 sq)
        {
            if (null == _tc)
                return;

            long refs = HitTestObstacle(sp, sq);
            _tc.RemoveObstacle(refs);
        }

        public long AddTempObstacle(Vector3 p)
        {
            if (null == _tc)
                return 0;

            p.Y -= 0.5f;
            return _tc.AddObstacle(p, 1.0f, 2.0f);
        }

        public DtTileCache GetTileCache()
        {
            return _tc;
        }

        public DtTileCache CreateTileCache(IInputGeomProvider geom, RcNavMeshBuildSettings setting, int tw, int th, RcByteOrder order, bool cCompatibility)
        {
            DtTileCacheParams option = new()
            {
                ch = setting.cellHeight,
                cs = setting.cellSize,
                orig = geom.GetMeshBoundsMin(),
                height = setting.tileSize,
                width = setting.tileSize,
                walkableHeight = setting.agentHeight,
                walkableRadius = setting.agentRadius,
                walkableClimb = setting.agentMaxClimb,
                maxSimplificationError = setting.edgeMaxError,
                maxTiles = tw * th * 4, // for test EXPECTED_LAYERS_PER_TILE;
                maxObstacles = 128
            };

            DtNavMeshParams navMeshParams = new()
            {
                orig = geom.GetMeshBoundsMin(),
                tileWidth = setting.tileSize * setting.cellSize,
                tileHeight = setting.tileSize * setting.cellSize,
                maxTiles = 256, // ..
                maxPolys = 16384
            };

            var navMesh = new DtNavMesh(navMeshParams, 6);
            var comp = _comp.Create(cCompatibility ? 0 : 1);
            var storageParams = new DtTileCacheStorageParams(order, cCompatibility);
            DtTileCache tc = new(option, storageParams, navMesh, comp, _proc);
            return tc;
        }

        public long HitTestObstacle(Vector3 sp, Vector3 sq)
        {
            float tmin = float.MaxValue;
            DtTileCacheObstacle obmin = null;

            for (int i = 0; i < _tc.GetObstacleCount(); ++i)
            {
                DtTileCacheObstacle ob = _tc.GetObstacle(i);
                if (ob.state == DtObstacleState.DT_OBSTACLE_EMPTY)
                    continue;

                Vector3 bmin = Vector3.Zero;
                Vector3 bmax = Vector3.Zero;
                DtTileCache.GetObstacleBounds(ob, ref bmin, ref bmax);


                if (Intersections.IsectSegAABB(sp, sq, bmin, bmax, out var t0, out _))
                {
                    if (t0 < tmin)
                    {
                        tmin = t0;
                        obmin = ob;
                    }
                }
            }

            return DtTileCache.GetObstacleRef(obmin);
        }
    }
}