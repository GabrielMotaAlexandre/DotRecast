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
using System.Numerics;
using System.Runtime.CompilerServices;
using UnityEngine;
using static System.Runtime.InteropServices.JavaScript.JSType;

namespace DotRecast.Detour
{
    public static class DtNavMeshBuilder
    {
        const int MESH_NULL_IDX = 0xffff;

        private static void CalcExtends(ReadOnlySpan<BVItem> items, int imin, int imax, out Vector3Int bmin, out Vector3Int bmax)
        {
            bmin = items[imin].bmin;
            bmax = items[imin].bmax;

            int minX = bmin.x;
            int minY = bmin.y;
            int minZ = bmin.z;

            int maxX = bmax.x;
            int maxY = bmax.y;
            int maxZ = bmax.z;

            for (int i = imin + 1; i < imax; ++i)
            {
                BVItem it = items[i];

                if (it.bmin.x < minX)
                    minX = it.bmin.x;
                if (it.bmin.y < minY)
                    minY = it.bmin.y;
                if (it.bmin.z < minZ)
                    minZ = it.bmin.z;

                if (it.bmax.x > maxX)
                    maxX = it.bmax.x;
                if (it.bmax.y > maxY)
                    maxY = it.bmax.y;
                if (it.bmax.z > maxZ)
                    maxZ = it.bmax.z;
            }

            bmin = new Vector3Int(minX, minY, minZ);
            bmax = new Vector3Int(maxX, maxY, maxZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int LongestAxis(in Vector3Int bmin, in Vector3Int bmax)
        {
            int axis = 0;
            int maxVal = bmax.x - bmin.x;
            var y = bmax.y - bmin.y;
            if (y > maxVal)
            {
                axis = 1;
                maxVal = y;
            }

            var z = bmax.z - bmin.z;
            if (z > maxVal)
            {
                axis = 2;
            }

            return axis;
        }

        public static int Subdivide(BVItem[] items, int nitems, int imin, int imax, int curNode, Span<DtBVNode> nodes)
        {
            int inum = imax - imin;
            int icur = curNode++;
            ref var node = ref nodes[icur];

            if (inum == 1)
            {
                // Leaf
                node = new DtBVNode(items[imin].bmin, items[imin].bmax, items[imin].i);
            }
            else
            {
                // Split
                CalcExtends(items, imin, imax, out var bmin, out var bmax);

                int axis = LongestAxis(in bmin, in bmax);

                if (axis is 0)
                {
                    // Sort along x-axis
                    Array.Sort(items, imin, inum, BVItemXComparer.Shared);
                }
                else if (axis == 1)
                {
                    // Sort along y-axis
                    Array.Sort(items, imin, inum, BVItemYComparer.Shared);
                }
                else
                {
                    // Sort along z-axis
                    Array.Sort(items, imin, inum, BVItemZComparer.Shared);
                }

                int isplit = imin + inum / 2;

                // Left
                curNode = Subdivide(items, nitems, imin, isplit, curNode, nodes);
                // Right
                curNode = Subdivide(items, nitems, isplit, imax, curNode, nodes);

                int iescape = curNode - icur;
                // Negative index means escape.
                node = new DtBVNode(bmin, bmax, -iescape);
            }

            return curNode;
        }

        private static int CreateBVTree(DtNavMeshCreateParams option, DtBVNode[] nodes)
        {
            // Build tree
            float quantFactor = 1 / option.cs;
            var items = new BVItem[option.polyCount];
            for (int i = 0; i < option.polyCount; i++)
            {
                ref var it = ref items[i];
                // Calc polygon bounds. Use detail meshes if available.
                if (option.detailMeshes != null)
                {
                    int vb = option.detailMeshes[i * 4];
                    int ndv = option.detailMeshes[i * 4 + 1];
                    Vector3 bmin = option.detailVerts[vb];
                    Vector3 bmax = bmin;

                    for (int j = 1; j < ndv; j++)
                    {
                        var vert = option.detailVerts[vb + j];
                        bmin.Min(vert);
                        bmax.Max(vert);
                    }

                    // BV-tree uses cs for all dimensions
                    var bminInt = new Vector3Int(Math.Clamp((int)((bmin.X - option.bmin.X) * quantFactor), 0, int.MaxValue),
                        Math.Clamp((int)((bmin.Y - option.bmin.Y) * quantFactor), 0, int.MaxValue),
                        Math.Clamp((int)((bmin.Z - option.bmin.Z) * quantFactor), 0, int.MaxValue));

                    var bmaxInt = new Vector3Int(Math.Clamp((int)((bmax.X - option.bmin.X) * quantFactor), 0, int.MaxValue),
                        Math.Clamp((int)((bmax.Y - option.bmin.Y) * quantFactor), 0, int.MaxValue),
                        Math.Clamp((int)((bmax.Z - option.bmin.Z) * quantFactor), 0, int.MaxValue));

                    it = new BVItem(bminInt, bmaxInt, i);
                }
                else
                {
                    int p = i * option.nvp * 2;
                    var bminX = option.verts[option.polys[p] * 3];
                    var bminY = option.verts[option.polys[p] * 3 + 1];
                    var bminZ = option.verts[option.polys[p] * 3 + 2];
                    var bmaxX = bminX;
                    var bmaxY = bminY;
                    var bmaxZ = bminZ;

                    for (int j = 1; j < option.nvp; ++j)
                    {
                        if (option.polys[p + j] == MESH_NULL_IDX)
                            break;

                        int x = option.verts[option.polys[p + j] * 3];
                        int y = option.verts[option.polys[p + j] * 3 + 1];
                        int z = option.verts[option.polys[p + j] * 3 + 2];

                        if (x < bminX)
                            bminX = x;
                        if (y < bminY)
                            bminY = y;
                        if (z < bminZ)
                            bminZ = z;

                        if (x > bmaxX)
                            bmaxX = x;
                        if (y > bmaxY)
                            bmaxY = y;
                        if (z > bmaxZ)
                            bmaxZ = z;
                    }

                    // Remap y
                    bminY = (int)MathF.Floor(bminY * option.ch * quantFactor);
                    bmaxY = (int)MathF.Ceiling(bmaxY * option.ch * quantFactor);

                    it = new BVItem(new Vector3Int(bminX, bminY, bminZ), new Vector3Int(bminX, bmaxY, bmaxZ), i);
                }
            }

            return Subdivide(items, option.polyCount, 0, option.polyCount, 0, nodes);
        }

        const int XP = 1 << 0;
        const int ZP = 1 << 1;
        const int XM = 1 << 2;
        const int ZM = 1 << 3;

        public static int ClassifyOffMeshPoint(Vector3 pt, Vector3 bmin, Vector3 bmax)
        {
            int outcode = 0;
            outcode |= (pt.X >= bmax.X) ? XP : 0;
            outcode |= (pt.Z >= bmax.Z) ? ZP : 0;
            outcode |= (pt.X < bmin.X) ? XM : 0;
            outcode |= (pt.Z < bmin.Z) ? ZM : 0;

            return outcode switch
            {
                XP => 0,
                XP | ZP => 1,
                ZP => 2,
                XM | ZP => 3,
                XM => 4,
                XM | ZM => 5,
                ZM => 6,
                XP | ZM => 7,
                _ => 0xff,
            };
        }

        /**
     * Builds navigation mesh tile data from the provided tile creation data.
     *
     * @param option
     *            Tile creation data.
     *
     * @return created tile data
     */
        public static DtMeshData CreateNavMeshData(DtNavMeshCreateParams option)
        {
            if (option.vertCount >= 0xffff)
                return null;
            if (option.vertCount is 0 || option.verts is null)
                return null;
            if (option.polyCount is 0 || option.polys is null)
                return null;

            int nvp = option.nvp;

            // Classify off-mesh connection points. We store only the connections
            // whose start point is inside the tile.
            int[] offMeshConClass = null;
            int storedOffMeshConCount = 0;
            int offMeshConLinkCount = 0;

            if (option.offMeshConCount > 0)
            {
                offMeshConClass = new int[option.offMeshConCount * 2];

                // Find tight heigh bounds, used for culling out off-mesh start
                // locations.
                float hmin = float.MaxValue;
                float hmax = -float.MaxValue;

                if (option.detailVerts != null && option.detailVerts.Length != 0)
                {
                    for (int i = 0; i < option.detailVerts.Length; ++i)
                    {
                        var h = option.detailVerts[i].Y;
                        hmin = MathF.Min(hmin, h);
                        hmax = MathF.Max(hmax, h);
                    }
                }
                else
                {
                    for (int i = 0; i < option.vertCount; ++i)
                    {
                        int iv = i * 3;
                        float h = option.bmin.Y + option.verts[iv + 1] * option.ch;
                        hmin = MathF.Min(hmin, h);
                        hmax = MathF.Max(hmax, h);
                    }
                }

                hmin -= option.walkableClimb;
                hmax += option.walkableClimb;
                Vector3 bmin = option.bmin;
                Vector3 bmax = option.bmax;
                bmin.Y = hmin;
                bmax.Y = hmax;

                for (int i = 0; i < option.offMeshConCount; ++i)
                {
                    var p0 = Vector3Extensions.Of(option.offMeshConVerts, (i * 2) * 3);
                    var p1 = Vector3Extensions.Of(option.offMeshConVerts, (i * 2 + 1) * 3);

                    offMeshConClass[i * 2] = ClassifyOffMeshPoint(p0, bmin, bmax);
                    offMeshConClass[i * 2 + 1] = ClassifyOffMeshPoint(p1, bmin, bmax);

                    // Zero out off-mesh start positions which are not even
                    // potentially touching the mesh.
                    if (offMeshConClass[i * 2] is 0xff)
                    {
                        if (p0.Y < bmin.Y || p0.Y > bmax.Y)
                            offMeshConClass[i * 2] = 0;
                    }

                    // Count how many links should be allocated for off-mesh
                    // connections.
                    if (offMeshConClass[i * 2] is 0xff)
                        offMeshConLinkCount++;
                    if (offMeshConClass[i * 2 + 1] is 0xff)
                        offMeshConLinkCount++;

                    if (offMeshConClass[i * 2] is 0xff)
                        storedOffMeshConCount++;
                }
            }

            // Off-mesh connections are stored as polygons, adjust values.
            int totPolyCount = option.polyCount + storedOffMeshConCount;
            int totVertCount = option.vertCount + storedOffMeshConCount * 2;

            // Find portal edges which are at tile borders.
            int edgeCount = 0;
            int portalCount = 0;
            for (int i = 0; i < option.polyCount; ++i)
            {
                int p = i * 2 * nvp;
                for (int j = 0; j < nvp; ++j)
                {
                    if (option.polys[p + j] == MESH_NULL_IDX)
                        break;
                    edgeCount++;

                    if ((option.polys[p + nvp + j] & 0x8000) != 0)
                    {
                        int dir = option.polys[p + nvp + j] & 0xf;
                        if (dir != 0xf)
                            portalCount++;
                    }
                }
            }

            int maxLinkCount = edgeCount + portalCount * 2 + offMeshConLinkCount * 2;

            // Find unique detail vertices.
            int uniqueDetailVertCount = 0;
            int detailTriCount;
            if (option.detailMeshes != null)
            {
                // Has detail mesh, count unique detail vertex count and use input
                // detail tri count.
                detailTriCount = option.detailTris.Length;
                for (int i = 0; i < option.polyCount; ++i)
                {
                    int p = i * nvp * 2;
                    int ndv = option.detailMeshes[i * 4 + 1];
                    int nv = 0;
                    for (int j = 0; j < nvp; ++j)
                    {
                        if (option.polys[p + j] == MESH_NULL_IDX)
                            break;
                        nv++;
                    }

                    ndv -= nv;
                    uniqueDetailVertCount += ndv;
                }
            }
            else
            {
                // No input detail mesh, build detail mesh from nav polys.
                uniqueDetailVertCount = 0; // No extra detail verts.
                detailTriCount = 0;
                for (int i = 0; i < option.polyCount; ++i)
                {
                    int p = i * nvp * 2;
                    int nv = 0;
                    for (int j = 0; j < nvp; ++j)
                    {
                        if (option.polys[p + j] == MESH_NULL_IDX)
                            break;
                        nv++;
                    }

                    detailTriCount += nv - 2;
                }
            }

            int bvTreeSize = option.buildBvTree ? option.polyCount * 2 : 0;
            DtMeshHeader header = new();
            // todoperf 
            var navVerts = new float[3 * totVertCount];
            var navPolys = new DtPoly[totPolyCount];
            var navDMeshes = new DtPolyDetail[option.polyCount];
            var navDVerts = new Vector3[uniqueDetailVertCount];
            var navDTris = new Vector4Int[detailTriCount];
            var navBvtree = new DtBVNode[bvTreeSize];
            var offMeshCons = new DtOffMeshConnection[storedOffMeshConCount];

            // Store header
            header.magic = DtMeshHeader.DT_NAVMESH_MAGIC;
            header.version = DtMeshHeader.DT_NAVMESH_VERSION;
            header.x = option.tileX;
            header.y = option.tileZ;
            header.layer = option.tileLayer;
            header.userId = option.userId;
            header.polyCount = totPolyCount;
            header.vertCount = totVertCount;
            header.maxLinkCount = maxLinkCount;
            header.bmin = option.bmin;
            header.bmax = option.bmax;
            header.detailMeshCount = option.polyCount;
            header.detailVertCount = uniqueDetailVertCount;
            header.detailTriCount = detailTriCount;
            header.bvQuantFactor = 1f / option.cs;
            header.offMeshBase = option.polyCount;
            header.walkableHeight = option.walkableHeight;
            header.walkableRadius = option.walkableRadius;
            header.walkableClimb = option.walkableClimb;
            header.offMeshConCount = storedOffMeshConCount;
            header.bvNodeCount = bvTreeSize;

            int offMeshVertsBase = option.vertCount;
            int offMeshPolyBase = option.polyCount;

            // Store vertices
            // Mesh vertices
            for (int i = 0; i < option.vertCount; ++i)
            {
                int iv = i * 3;
                int v = i * 3;
                navVerts[v] = option.bmin.X + option.verts[iv] * option.cs;
                navVerts[v + 1] = option.bmin.Y + option.verts[iv + 1] * option.ch;
                navVerts[v + 2] = option.bmin.Z + option.verts[iv + 2] * option.cs;
            }

            // Off-mesh link vertices.
            int n = 0;
            for (int i = 0; i < option.offMeshConCount; ++i)
            {
                // Only store connections which start from this tile.
                if (offMeshConClass[i * 2] is 0xff)
                {
                    int linkv = i * 2 * 3;
                    int v = (offMeshVertsBase + n * 2) * 3;
                    Array.Copy(option.offMeshConVerts, linkv, navVerts, v, 6);
                    n++;
                }
            }

            // Store polygons
            // Mesh polys
            int src = 0;
            for (int i = 0; i < option.polyCount; ++i)
            {
                DtPoly p = new(i, nvp);
                navPolys[i] = p;
                p.vertCount = 0;
                p.flags = option.polyFlags[i];
                p.SetArea(option.polyAreas[i]);
                p.SetPolyType(DtPoly.DT_POLYTYPE_GROUND);
                for (int j = 0; j < nvp; ++j)
                {
                    if (option.polys[src + j] == MESH_NULL_IDX)
                        break;
                    p.verts[j] = option.polys[src + j];
                    if ((option.polys[src + nvp + j] & 0x8000) != 0)
                    {
                        // Border or portal edge.
                        int dir = option.polys[src + nvp + j] & 0xf;
                        if (dir is 0xf) // Border
                            p.neis[j] = 0;
                        else if (dir is 0) // Portal x-
                            p.neis[j] = DtNavMesh.DT_EXT_LINK | 4;
                        else if (dir == 1) // Portal z+
                            p.neis[j] = DtNavMesh.DT_EXT_LINK | 2;
                        else if (dir == 2) // Portal x+
                            p.neis[j] = DtNavMesh.DT_EXT_LINK | 0;
                        else if (dir == 3) // Portal z-
                            p.neis[j] = DtNavMesh.DT_EXT_LINK | 6;
                    }
                    else
                    {
                        // Normal connection
                        p.neis[j] = option.polys[src + nvp + j] + 1;
                    }

                    p.vertCount++;
                }

                src += nvp * 2;
            }

            // Off-mesh connection vertices.
            n = 0;
            for (int i = 0; i < option.offMeshConCount; ++i)
            {
                // Only store connections which start from this tile.
                if (offMeshConClass[i * 2] is 0xff)
                {
                    DtPoly p = new(offMeshPolyBase + n, nvp);
                    navPolys[offMeshPolyBase + n] = p;
                    p.vertCount = 2;
                    p.verts[0] = offMeshVertsBase + n * 2;
                    p.verts[1] = offMeshVertsBase + n * 2 + 1;
                    p.flags = option.offMeshConFlags[i];
                    p.SetArea(option.offMeshConAreas[i]);
                    p.SetPolyType(DtPoly.DT_POLYTYPE_OFFMESH_CONNECTION);
                    n++;
                }
            }

            // Store detail meshes and vertices.
            // The nav polygon vertices are stored as the first vertices on each
            // mesh.
            // We compress the mesh data by skipping them and using the navmesh
            // coordinates.
            if (option.detailMeshes != null)
            {
                int vbase = 0;
                for (int i = 0; i < option.polyCount; ++i)
                {
                    int vb = option.detailMeshes[i * 4];
                    int ndv = option.detailMeshes[i * 4 + 1];
                    int nv = navPolys[i].vertCount;

                    navDMeshes[i] = new(vbase, option.detailMeshes[i * 4 + 2], ndv - nv, option.detailMeshes[i * 4 + 3]);

                    // Copy vertices except the first 'nv' verts which are equal to
                    // nav poly verts.
                    if (ndv - nv != 0)
                    {
                        Array.Copy(option.detailVerts, vb + nv, navDVerts, vbase, ndv - nv);
                        vbase += ndv - nv;
                    }
                }

                // Store triangles.
                Array.Copy(option.detailTris, 0, navDTris, 0, option.detailTris.Length);
            }
            else
            {
                // Create dummy detail mesh by triangulating polys.
                int tbase = 0;
                for (int i = 0; i < option.polyCount; ++i)
                {
                    int nv = navPolys[i].vertCount;
                    navDMeshes[i] = new(0, tbase, 0, nv - 2);

                    // Triangulate polygon (local indices).
                    for (int j = 2; j < nv; ++j)
                    {
                        int t = tbase * 4;

                        // Bit for each edge that belongs to poly boundary.
                        const int edge = 1 << 2;
                        int w = edge;
                        if (j == 2)
                            w |= 1 << 0;
                        if (j == nv - 1)
                            w |= 1 << 4;

                        navDTris[t] = new Vector4Int(0, j - 1, j, w);

                        tbase++;
                    }
                }
            }

            // Store and create BVtree.
            // TODO: take detail mesh into account! use byte per bbox extent?
            if (option.buildBvTree)
            {
                // Do not set header.bvNodeCount set to make it work look exactly the same as in original Detour
                header.bvNodeCount = CreateBVTree(option, navBvtree);
            }

            // Store Off-Mesh connections.
            n = 0;
            for (int i = 0; i < option.offMeshConCount; ++i)
            {
                // Only store connections which start from this tile.
                if (offMeshConClass[i * 2] is 0xff)
                {
                    DtOffMeshConnection con = new();
                    offMeshCons[n] = con;
                    con.poly = offMeshPolyBase + n;
                    // Copy connection end-points.
                    int endPts = i * 2 * 3;
                    Array.Copy(option.offMeshConVerts, endPts, con.pos, 0, 6);
                    con.rad = option.offMeshConRad[i];
                    con.flags = option.offMeshConDir[i] != 0 ? DtNavMesh.DT_OFFMESH_CON_BIDIR : 0;
                    con.side = offMeshConClass[i * 2 + 1];
                    if (option.offMeshConUserID != null)
                        con.userId = option.offMeshConUserID[i];
                    n++;
                }
            }

            DtMeshData nmd = new()
            {
                header = header,
                verts = navVerts,
                polys = navPolys,
                detailMeshes = navDMeshes,
                detailVerts = navDVerts,
                detailTris = navDTris,
                bvTree = navBvtree,
                offMeshCons = offMeshCons
            };
            return nmd;
        }
    }
}