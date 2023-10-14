/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org
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
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using DotRecast.Core;
using DotRecast.Detour.Dynamic.Colliders;
using DotRecast.Detour.Dynamic.Io;
using DotRecast.Recast;

namespace DotRecast.Detour.Dynamic
{
    public class DtDynamicNavMesh
    {
        public const int MAX_VERTS_PER_POLY = 6;
        public readonly DtDynamicNavMeshConfig config;
        private readonly RcBuilder builder;
        private readonly Dictionary<long, DtDynamicTile> _tiles = new();
        private readonly DtNavMeshParams navMeshParams;
        private readonly BlockingCollection<IDtDaynmicTileJob> updateQueue = new();
        private readonly RcAtomicLong currentColliderId = new(0);
        private DtNavMesh _navMesh;
        private bool dirty = true;

        public DtDynamicNavMesh(DtVoxelFile voxelFile)
        {
            config = new DtDynamicNavMeshConfig(voxelFile.useTiles, voxelFile.tileSizeX, voxelFile.tileSizeZ, voxelFile.cellSize)
            {
                walkableHeight = voxelFile.walkableHeight,
                walkableRadius = voxelFile.walkableRadius,
                walkableClimb = voxelFile.walkableClimb,
                walkableSlopeAngle = voxelFile.walkableSlopeAngle,
                maxSimplificationError = voxelFile.maxSimplificationError,
                maxEdgeLen = voxelFile.maxEdgeLen,
                minRegionArea = voxelFile.minRegionArea,
                regionMergeArea = voxelFile.regionMergeArea,
                vertsPerPoly = voxelFile.vertsPerPoly,
                buildDetailMesh = voxelFile.buildMeshDetail,
                detailSampleDistance = voxelFile.detailSampleDistance,
                detailSampleMaxError = voxelFile.detailSampleMaxError
            };
            builder = new RcBuilder();
            navMeshParams = new DtNavMeshParams();
            navMeshParams.orig.X = voxelFile.bounds[0];
            navMeshParams.orig.Y = voxelFile.bounds[2];
            //navMeshParams.orig.Z = voxelFile.bounds[2];
            navMeshParams.tileWidth = voxelFile.cellSize * voxelFile.tileSizeX;
            navMeshParams.tileHeight = voxelFile.cellSize * voxelFile.tileSizeZ;
            navMeshParams.maxTiles = voxelFile.tiles.Count;
            navMeshParams.maxPolys = 0x8000;
            foreach (var t in voxelFile.tiles)
            {
                _tiles.Add(LookupKey(t.tileX, t.tileZ), new DtDynamicTile(t));
            }
        }

        public DtNavMesh NavMesh()
        {
            return _navMesh;
        }

        /**
     * Voxel queries require checkpoints to be enabled in {@link DynamicNavMeshConfig}
     */
        public DtVoxelQuery VoxelQuery()
        {
            return new DtVoxelQuery(navMeshParams.orig, navMeshParams.tileWidth, navMeshParams.tileHeight, LookupHeightfield);
        }

        private RcHeightfield LookupHeightfield(int x, int z)
        {
            return GetTileAt(x, z)?.checkpoint.heightfield ?? default;
        }

        public long AddCollider(IDtCollider collider)
        {
            long cid = currentColliderId.IncrementAndGet();
            updateQueue.Add(new DtDynamicTileColliderAdditionJob(cid, collider, GetTiles(collider.Bounds())));
            return cid;
        }

        public void RemoveCollider(long colliderId)
        {
            updateQueue.Add(new DtDynamicTileColliderRemovalJob(colliderId, GetTilesByCollider(colliderId)));
        }

        /**
     * Perform full build of the nav mesh
     */
        public void Build()
        {
            ProcessQueue();
            Rebuild(_tiles.Values);
        }

        /**
     * Perform incremental update of the nav mesh
     */
        public bool Update()
        {
            return Rebuild(ProcessQueue());
        }

        private bool Rebuild(ICollection<DtDynamicTile> stream)
        {
            foreach (var dynamicTile in stream)
                Rebuild(dynamicTile);
            return UpdateNavMesh();
        }

        private HashSet<DtDynamicTile> ProcessQueue()
        {
            var items = ConsumeQueue();
            foreach (var item in items)
            {
                Process(item);
            }

            return items.SelectMany(i => i.AffectedTiles()).ToHashSet();
        }

        private List<IDtDaynmicTileJob> ConsumeQueue()
        {
            List<IDtDaynmicTileJob> items = new();
            while (updateQueue.TryTake(out var item))
            {
                items.Add(item);
            }

            return items;
        }

        private static void Process(IDtDaynmicTileJob item)
        {
            foreach (var tile in item.AffectedTiles())
            {
                item.Process(tile);
            }
        }

        /**
     * Perform full build concurrently using the given {@link ExecutorService}
     */
        public Task<bool> Build(TaskFactory executor)
        {
            ProcessQueue();
            return Rebuild(_tiles.Values, executor);
        }

        /**
     * Perform incremental update concurrently using the given {@link ExecutorService}
     */
        public Task<bool> Update(TaskFactory executor)
        {
            return Rebuild(ProcessQueue(), executor);
        }

        private Task<bool> Rebuild(ICollection<DtDynamicTile> tiles, TaskFactory executor)
        {
            var tasks = tiles.Select(tile => executor.StartNew(() => Rebuild(tile))).ToArray();
            return Task.WhenAll(tasks).ContinueWith(k => UpdateNavMesh());
        }

        private ICollection<DtDynamicTile> GetTiles(float[] bounds)
        {
            if (bounds == null)
            {
                return _tiles.Values;
            }

            int minx = (int)Math.Floor((bounds[0] - navMeshParams.orig.X) / navMeshParams.tileWidth);
            int minz = (int)Math.Floor((bounds[2] - navMeshParams.orig.Y) / navMeshParams.tileHeight);
            int maxx = (int)Math.Floor((bounds[3] - navMeshParams.orig.X) / navMeshParams.tileWidth);
            int maxz = (int)Math.Floor((bounds[5] - navMeshParams.orig.Y) / navMeshParams.tileHeight);
            List<DtDynamicTile> tiles = new();
            for (int z = minz; z <= maxz; ++z)
            {
                for (int x = minx; x <= maxx; ++x)
                {
                    DtDynamicTile tile = GetTileAt(x, z);
                    if (tile != null)
                    {
                        tiles.Add(tile);
                    }
                }
            }

            return tiles;
        }

        private List<DtDynamicTile> GetTilesByCollider(long cid)
        {
            return _tiles.Values.Where(t => t.ContainsCollider(cid)).ToList();
        }

        private void Rebuild(DtDynamicTile tile)
        {
            DtNavMeshCreateParams option = new()
            {
                walkableHeight = config.walkableHeight
            };
            dirty |= tile.Build(builder, config);
        }

        private bool UpdateNavMesh()
        {
            if (dirty)
            {
                DtNavMesh navMesh = new(navMeshParams, MAX_VERTS_PER_POLY);
                foreach (var t in _tiles.Values)
                    t.AddTo(navMesh);

                this._navMesh = navMesh;
                dirty = false;
                return true;
            }

            return false;
        }

        private DtDynamicTile GetTileAt(int x, int z)
        {
            return _tiles.TryGetValue(LookupKey(x, z), out var tile)
                ? tile
                : null;
        }

        private static long LookupKey(long x, long z)
        {
            return (z << 32) | x;
        }

        public List<DtVoxelTile> VoxelTiles()
        {
            return _tiles.Values.Select(t => t.voxelTile).ToList();
        }

        public List<RcBuilderResult> RecastResults()
        {
            return _tiles.Values.Select(t => t.recastResult).ToList();
        }
    }
}