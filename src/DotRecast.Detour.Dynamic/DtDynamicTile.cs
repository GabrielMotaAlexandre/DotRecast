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
using System.Numerics;
using System.Runtime.InteropServices;
using DotRecast.Core;
using DotRecast.Detour.Dynamic.Colliders;
using DotRecast.Detour.Dynamic.Io;
using DotRecast.Recast;
using UnityEngine;

namespace DotRecast.Detour.Dynamic
{
    public class DtDynamicTile
    {
        public readonly DtVoxelTile voxelTile;
        public DtDynamicTileCheckpoint checkpoint;
        public RcBuilderResult recastResult;
        private DtMeshData meshData;
        private readonly ConcurrentDictionary<long, IDtCollider> colliders = new();
        private bool dirty = true;
        private long id;

        public DtDynamicTile(DtVoxelTile voxelTile)
        {
            this.voxelTile = voxelTile;
        }

        public bool Build(DtDynamicNavMeshConfig config)
        {
            if (dirty)
            {
                RcHeightfield heightfield = BuildHeightfield(config);
                RcBuilderResult r = BuildRecast(config, voxelTile, heightfield);
                DtNavMeshCreateParams option = NavMeshCreateParams(voxelTile.tileX, voxelTile.tileZ, voxelTile.cellSize,
                    voxelTile.cellHeight, config, r);
                meshData = DtNavMeshBuilder.CreateNavMeshData(option);
                return true;
            }

            return false;
        }

        private RcHeightfield BuildHeightfield(DtDynamicNavMeshConfig config)
        {
            ICollection<long> rasterizedColliders = checkpoint != null
                ? checkpoint.colliders
                : Array.Empty<long>();

            RcHeightfield source = checkpoint != null
                ? checkpoint.heightfield
                : voxelTile.Heightfield();

            var bmaxY = source.bmax.Y;
            foreach (var (cid, c) in colliders)
            {
                if (!rasterizedColliders.Contains(cid))
                {
                    bmaxY = Math.Max(source.bmax.Y, c.Bounds()[4] + source.ch * 2);
                    c.Rasterize(source);
                }
            }

            var heightfield = new RcHeightfield(source.width, source.height, source.bmin, new System.Numerics.Vector3(source.bmax.X, bmaxY, source.bmax.Z), source.cs, source.ch, source.borderSize, source.spans);

            if (config.enableCheckpoints)
            {
                checkpoint = new DtDynamicTileCheckpoint(heightfield, colliders.Keys.ToHashSet());
            }

            return heightfield;
        }

        private RcBuilderResult BuildRecast(DtDynamicNavMeshConfig config, DtVoxelTile vt, in RcHeightfield heightfield)
        {
            RcConfig rcConfig = new(
                config.useTiles, config.tileSizeX, config.tileSizeZ,
                vt.borderSize,
                config.partition,
                vt.cellSize, vt.cellHeight,
                config.walkableSlopeAngle, config.walkableHeight, config.walkableRadius, config.walkableClimb,
                config.minRegionArea, config.regionMergeArea,
                config.maxEdgeLen, config.maxSimplificationError,
                Math.Min(DtDynamicNavMesh.MAX_VERTS_PER_POLY, config.vertsPerPoly),
                config.detailSampleDistance, config.detailSampleMaxError,
                true, true, true, default, true);
            RcBuilderResult r = RcBuilder.Build(vt.tileX, vt.tileZ, null, rcConfig, heightfield);
            if (config.keepIntermediateResults)
            {
                recastResult = r;
            }

            return r;
        }

        public void AddCollider(long cid, IDtCollider collider)
        {
            colliders[cid] = collider;
            dirty = true;
        }

        public bool ContainsCollider(long cid)
        {
            return colliders.ContainsKey(cid);
        }

        public void RemoveCollider(long colliderId)
        {
            if (colliders.TryRemove(colliderId, out _))
            {
                dirty = true;
                checkpoint = null;
            }
        }

        private static DtNavMeshCreateParams NavMeshCreateParams(int tilex, int tileZ, float cellSize, float cellHeight,
            DtDynamicNavMeshConfig config, RcBuilderResult rcResult)
        {
            RcPolyMesh m_pmesh = rcResult.Mesh;
            RcPolyMeshDetail m_dmesh = rcResult.MeshDetail;
            DtNavMeshCreateParams option = new();
            for (int i = 0; i < m_pmesh.npolys; ++i)
            {
                m_pmesh.flags[i] = 1;
            }

            option.tileX = tilex;
            option.tileZ = tileZ;
            option.verts = m_pmesh.verts;
            option.vertCount = m_pmesh.nverts;
            option.polys = m_pmesh.polys;
            option.polyAreas = m_pmesh.areas;
            option.polyFlags = m_pmesh.flags;
            option.polyCount = m_pmesh.npolys;
            option.nvp = m_pmesh.nvp;
            if (m_dmesh.IsValid)
            {
                option.detailMeshes = m_dmesh.meshes;
                option.detailVerts = m_dmesh.verts;
                option.detailTris = m_dmesh.tris;
            }

            option.walkableHeight = config.walkableHeight;
            option.walkableRadius = config.walkableRadius;
            option.walkableClimb = config.walkableClimb;
            option.bmin = m_pmesh.bmin;
            option.bmax = m_pmesh.bmax;
            option.cs = cellSize;
            option.ch = cellHeight;
            option.buildBvTree = true;

            option.offMeshConCount = 0;
            option.offMeshConVerts = Array.Empty<float>();
            option.offMeshConRad = Array.Empty<float>();
            option.offMeshConDir = Array.Empty<int>();
            option.offMeshConAreas = Array.Empty<int>();
            option.offMeshConFlags = Array.Empty<int>();
            option.offMeshConUserID = Array.Empty<int>();
            return option;
        }

        public void AddTo(DtNavMesh navMesh)
        {
            if (meshData != null)
            {
                id = navMesh.AddTile(meshData, 0, 0);
            }
            else
            {
                navMesh.RemoveTile(id);
                id = 0;
            }
        }
    }
}