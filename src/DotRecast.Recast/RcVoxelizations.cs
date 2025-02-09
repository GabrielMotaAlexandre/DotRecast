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
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using DotRecast.Recast.Geom;

namespace DotRecast.Recast
{
    public static class RcVoxelizations
    {
        public static RcHeightfield BuildSolidHeightfield(IInputGeomProvider geomProvider, in RcBuilderConfig builderCfg)
        {
            RcConfig cfg = builderCfg.cfg;

            // Allocate voxel heightfield where we rasterize our input data to.
            RcHeightfield solid = new(builderCfg.width, builderCfg.height, builderCfg.bmin, builderCfg.bmax, cfg.Cs, cfg.Ch, cfg.BorderSize);
            var bmin = builderCfg.bmin.AsVector2XZ();
            var bmax = builderCfg.bmax.AsVector2XZ();
            // Allocate array that can hold triangle area types.
            // If you have multiple meshes you need to process, allocate
            // and array which can hold the max number of triangles you need to
            // process.

            // Find triangles which are walkable based on their slope and rasterize
            // them.
            // If your input data is multiple meshes, you can transform them here,
            // calculate
            // the are type for each of the meshes and rasterize them.
            foreach (RcTriMesh geom in geomProvider.Meshes())
            {
                var verts = MemoryMarshal.Cast<float, Vector3>(geom.Vertices.AsSpan());
                IEnumerable<int[]> nodes;

                if (cfg.UseTiles)
                {
                    nodes = geom.GetChunksOverlappingRect(bmin, bmax).Select(x => x.tris);
                }
                else
                {
                    nodes = new int[][] { geom.Triangles };
                }

                foreach (var triangles in nodes)
                {
                    var m_triareas = RcCommons.MarkWalkableTriangles(cfg.WalkableSlopeAngle, verts, triangles, cfg.WalkableAreaMod);
                    RcRasterizations.RasterizeTriangles(in solid, verts, triangles, m_triareas, cfg.WalkableClimb);
                }
            }

            return solid;
        }
    }
}