/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org
DotRecast Copyright (c) 2023 Choi Ikpil ikpil@naver.com

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using DotRecast.Core;
using DotRecast.Recast.Geom;

namespace DotRecast.Recast
{
    using static RcCommons;
    using static RcAreas;

    public static class RcBuilder
    {
        public static List<RcBuilderResult> BuildTiles(IInputGeomProvider geom, RcConfig cfg)
        {
            var bmin = geom.GetMeshBoundsMin();
            var bmax = geom.GetMeshBoundsMax();
            CalcTileCount(bmin.AsVector2XZ(), bmax.AsVector2XZ(), cfg.Cs, cfg.TileSizeX, cfg.TileSizeZ, out var tw, out var th);

            BuildSingleThreadAsync(geom, cfg, bmin, bmax, tw, th, out var results);

            return results;
        }


        public static Task BuildTilesAsync(IInputGeomProvider geom, RcConfig cfg, out List<RcBuilderResult> results)
        {
            var bmin = geom.GetMeshBoundsMin();
            var bmax = geom.GetMeshBoundsMax();
            CalcTileCount(bmin.AsVector2XZ(), bmax.AsVector2XZ(), cfg.Cs, cfg.TileSizeX, cfg.TileSizeZ, out var tw, out var th);

            return BuildMultiThreadAsync(geom, cfg, bmin, bmax, tw, th, out results);
        }

        private static Task BuildSingleThreadAsync(IInputGeomProvider geom, RcConfig cfg, Vector3 bmin, Vector3 bmax,
            int tw, int th, out List<RcBuilderResult> results)
        {
            results = new List<RcBuilderResult>();
            for (int y = 0; y < th; ++y)
            {
                for (int x = 0; x < tw; ++x)
                {
                    results.Add(BuildTile(geom, cfg, bmin, bmax, x, y));
                }
            }

            return Task.CompletedTask;
        }

        private static Task BuildMultiThreadAsync(IInputGeomProvider geom, RcConfig cfg, Vector3 bmin, Vector3 bmax, int tw, int th, out List<RcBuilderResult> results)
        {
            var resultsLocal = new List<RcBuilderResult>();
            CountdownEvent latch = new(tw * th);
            List<Task> tasks = new();

            for (int x = 0; x < tw; ++x)
            {
                for (int y = 0; y < th; ++y)
                {
                    int tx = x;
                    int ty = y;
                    var task = Task.Run(() =>
                    {
                        try
                        {
                            RcBuilderResult tile = BuildTile(geom, cfg, bmin, bmax, tx, ty);
                            lock (resultsLocal)
                            {
                                resultsLocal.Add(tile);
                            }
                        }
                        catch (Exception e)
                        {
                            Console.WriteLine(e);
                        }

                        latch.Signal();
                    });

                    tasks.Add(task);
                }
            }

            try
            {
                latch.Wait();
            }
            catch (ThreadInterruptedException)
            {
            }
            results = resultsLocal;
            return Task.WhenAll(tasks);
        }

        public static RcBuilderResult BuildTile(IInputGeomProvider geom, RcConfig cfg, Vector3 bmin, Vector3 bmax, int tx, int ty)
        {
            return Build(geom, new RcBuilderConfig(cfg, bmin, bmax, tx, ty));
        }

        public static RcBuilderResult Build(IInputGeomProvider geom, in RcBuilderConfig builderCfg)
        {
            RcConfig cfg = builderCfg.cfg;
            //
            // Step 1. Rasterize input polygon soup.
            //
            RcHeightfield solid = RcVoxelizations.BuildSolidHeightfield(geom, in builderCfg);
            return Build(builderCfg.tileX, builderCfg.tileZ, geom, cfg, in solid);
        }

        public static RcBuilderResult Build(int tileX, int tileZ, IInputGeomProvider geom, RcConfig cfg, in RcHeightfield solid)
        {
            FilterHeightfield(in solid, cfg);
            RcCompactHeightfield chf = BuildCompactHeightfield(geom, cfg, in solid);

            // Partition the heightfield so that we can use simple algorithm later
            // to triangulate the walkable areas.
            // There are 3 partitioning methods, each with some pros and cons:
            // 1) Watershed partitioning
            // - the classic Recast partitioning
            // - creates the nicest tessellation
            // - usually slowest
            // - partitions the heightfield into nice regions without holes or
            // overlaps
            // - the are some corner cases where this method creates produces holes
            // and overlaps
            // - holes may appear when a small obstacles is close to large open area
            // (triangulation can handle this)
            // - overlaps may occur if you have narrow spiral corridors (i.e
            // stairs), this make triangulation to fail
            // * generally the best choice if you precompute the navmesh, use this
            // if you have large open areas
            // 2) Monotone partioning
            // - fastest
            // - partitions the heightfield into regions without holes and overlaps
            // (guaranteed)
            // - creates long thin polygons, which sometimes causes paths with
            // detours
            // * use this if you want fast navmesh generation
            // 3) Layer partitoining
            // - quite fast
            // - partitions the heighfield into non-overlapping regions
            // - relies on the triangulation code to cope with holes (thus slower
            // than monotone partitioning)
            // - produces better triangles than monotone partitioning
            // - does not have the corner cases of watershed partitioning
            // - can be slow and create a bit ugly tessellation (still better than
            // monotone)
            // if you have large open areas with small obstacles (not a problem if
            // you use tiles)
            // * good choice to use for tiled navmesh with medium and small sized
            // tiles

            if (cfg.Partition == RcPartition.WATERSHED)
            {
                // Prepare for region partitioning, by calculating distance field
                // along the walkable surface.
                RcRegions.BuildDistanceField(ref chf);
                // Partition the walkable surface into simple regions without holes.
                RcRegions.BuildRegions(ref chf, cfg.MinRegionArea, cfg.MergeRegionArea);
            }
            else if (cfg.Partition == RcPartition.MONOTONE)
            {
                // Partition the walkable surface into simple regions without holes.
                // Monotone partitioning does not need distancefield.
                RcRegions.BuildRegions(ref chf, cfg.MinRegionArea, false, cfg.MergeRegionArea);
            }
            else
            {
                // Partition the walkable surface into simple regions without holes.
                RcRegions.BuildRegions(ref chf, cfg.MinRegionArea, true);
            }

            //
            // Step 5. Trace and simplify region contours.
            //

            // Create contours.
            RcContourSet cset = RcContours.BuildContours(in chf, cfg.MaxSimplificationError, cfg.MaxEdgeLen, RcConstants.RC_CONTOUR_TESS_WALL_EDGES);

            //
            // Step 6. Build polygons mesh from contours.
            //

            RcPolyMesh pmesh = RcMeshs.BuildPolyMesh(cset, cfg.MaxVertsPerPoly);

            //
            // Step 7. Create detail mesh which allows to access approximate height
            // on each polygon.
            //
            RcPolyMeshDetail dmesh = cfg.BuildMeshDetail
                ? RcMeshDetails.BuildPolyMeshDetail(pmesh, in chf, cfg.DetailSampleDist, cfg.DetailSampleMaxError)
                : null;
            return new RcBuilderResult(tileX, tileZ, solid, chf, cset, pmesh, dmesh);
        }

        /*
         * Step 2. Filter walkable surfaces.
         */
        private static void FilterHeightfield(in RcHeightfield solid, RcConfig cfg)
        {
            // Once all geometry is rasterized, we do initial pass of filtering to
            // remove unwanted overhangs caused by the conservative rasterization
            // as well as filter spans where the character cannot possibly stand.
            if (cfg.FilterLowHangingObstacles)
            {
                RcFilters.FilterLowHangingWalkableObstacles(cfg.WalkableClimb, in solid);
            }

            if (cfg.FilterLedgeSpans)
            {
                RcFilters.FilterLedgeSpans(cfg.WalkableHeight, cfg.WalkableClimb, in solid);
            }

            if (cfg.FilterWalkableLowHeightSpans)
            {
                RcFilters.FilterWalkableLowHeightSpans(cfg.WalkableHeight, in solid);
            }
        }

        /*
         * Step 3. Partition walkable surface to simple regions.
         */
        private static RcCompactHeightfield BuildCompactHeightfield(IInputGeomProvider geom, RcConfig cfg, in RcHeightfield solid)
        {
            // Compact the heightfield so that it is faster to handle from now on.
            // This will result more cache coherent data as well as the neighbours
            // between walkable cells will be calculated.
            RcCompactHeightfield chf = new(in solid, in cfg);

            // Erode the walkable area by agent radius.
            ErodeWalkableArea(cfg.WalkableRadius, chf);
            // (Optional) Mark areas.
            if (geom != null)
            {
                foreach (RcConvexVolume vol in geom.ConvexVolumes())
                {
                    MarkConvexPolyArea(vol.verts, vol.hmin, vol.hmax, vol.areaMod, chf);
                }
            }

            return chf;
        }

        public static RcHeightfieldLayerSet BuildLayers(IInputGeomProvider geom, RcBuilderConfig builderCfg)
        {
            RcHeightfield solid = RcVoxelizations.BuildSolidHeightfield(geom, builderCfg);
            FilterHeightfield(in solid, builderCfg.cfg);
            RcCompactHeightfield chf = BuildCompactHeightfield(geom, builderCfg.cfg, in solid);
            return RcLayers.BuildHeightfieldLayers(chf, builderCfg.cfg.WalkableHeight);
        }
    }
}