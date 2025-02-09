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
using System.Runtime.CompilerServices;

namespace DotRecast.Recast.Geom
{
    public class RcChunkyTriMesh
    {
        private readonly List<RcChunkyTriMeshNode> nodes;
        private readonly int maxTrisPerChunk;

        private static void CalcExtends(BoundsItem[] items, int imin, int imax, ref Vector2 bmin, ref Vector2 bmax)
        {
            bmin.X = items[imin].bmin.X;
            bmin.Y = items[imin].bmin.Y;

            bmax.X = items[imin].bmax.X;
            bmax.Y = items[imin].bmax.Y;

            for (int i = imin + 1; i < imax; ++i)
            {
                BoundsItem it = items[i];
                if (it.bmin.X < bmin.X)
                {
                    bmin.X = it.bmin.X;
                }

                if (it.bmin.Y < bmin.Y)
                {
                    bmin.Y = it.bmin.Y;
                }

                if (it.bmax.X > bmax.X)
                {
                    bmax.X = it.bmax.X;
                }

                if (it.bmax.Y > bmax.Y)
                {
                    bmax.Y = it.bmax.Y;
                }
            }
        }

        private static int LongestAxis(float x, float y)
        {
            return y > x ? 1 : 0;
        }

        private static void Subdivide(BoundsItem[] items, int imin, int imax, int trisPerChunk, List<RcChunkyTriMeshNode> nodes, int[] inTris)
        {
            int inum = imax - imin;

            RcChunkyTriMeshNode node = new();
            nodes.Add(node);

            if (inum <= trisPerChunk)
            {
                // Leaf
                CalcExtends(items, imin, imax, ref node.bmin, ref node.bmax);

                // Copy triangles.
                node.i = nodes.Count;
                node.tris = new int[inum * 3];

                int dst = 0;
                for (int i = imin; i < imax; ++i)
                {
                    int src = items[i].i * 3;
                    node.tris[dst++] = inTris[src];
                    node.tris[dst++] = inTris[src + 1];
                    node.tris[dst++] = inTris[src + 2];
                }
            }
            else
            {
                // Split
                CalcExtends(items, imin, imax, ref node.bmin, ref node.bmax);

                int axis = LongestAxis(node.bmax.X - node.bmin.X, node.bmax.Y - node.bmin.Y);

                if (axis is 0)
                {
                    Array.Sort(items, imin, imax - imin, BoundsItemXComparer.Shared);
                    // Sort along x-axis
                }
                else if (axis == 1)
                {
                    Array.Sort(items, imin, imax - imin, BoundsItemYComparer.Shared);
                    // Sort along y-axis
                }

                int isplit = imin + inum / 2;

                // Left
                Subdivide(items, imin, isplit, trisPerChunk, nodes, inTris);
                // Right
                Subdivide(items, isplit, imax, trisPerChunk, nodes, inTris);

                // Negative index means escape.
                node.i = -nodes.Count;
            }
        }

        public RcChunkyTriMesh(float[] verts, int[] tris, int ntris, int trisPerChunk)
        {
            int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

            nodes = new List<RcChunkyTriMeshNode>(nchunks);

            // Build tree
            BoundsItem[] items = new BoundsItem[ntris];

            for (int i = 0; i < ntris; i++)
            {
                int t = i * 3;
                BoundsItem it = items[i] = new BoundsItem();
                it.i = i;
                // Calc triangle XZ bounds.
                it.bmin.X = it.bmax.X = verts[tris[t] * 3];
                it.bmin.Y = it.bmax.Y = verts[tris[t] * 3 + 2];
                for (int j = 1; j < 3; ++j)
                {
                    int v = tris[t + j] * 3;
                    if (verts[v] < it.bmin.X)
                    {
                        it.bmin.X = verts[v];
                    }

                    if (verts[v + 2] < it.bmin.Y)
                    {
                        it.bmin.Y = verts[v + 2];
                    }

                    if (verts[v] > it.bmax.X)
                    {
                        it.bmax.X = verts[v];
                    }

                    if (verts[v + 2] > it.bmax.Y)
                    {
                        it.bmax.Y = verts[v + 2];
                    }
                }
            }

            Subdivide(items, 0, ntris, trisPerChunk, nodes, tris);

            // Calc max tris per node.
            maxTrisPerChunk = 0;
            foreach (RcChunkyTriMeshNode node in nodes)
            {
                bool isLeaf = node.i >= 0;
                if (!isLeaf)
                {
                    continue;
                }

                if (node.tris.Length / 3 > maxTrisPerChunk)
                {
                    maxTrisPerChunk = node.tris.Length / 3;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool CheckOverlapRect(Vector2 amin, Vector2 amax, Vector2 bmin, Vector2 bmax)
        {
            return amin.X <= bmax.X && amax.X >= bmin.X && amin.Y <= bmax.Y && amax.Y >= bmin.Y;
        }

        public List<RcChunkyTriMeshNode> GetChunksOverlappingRect(Vector2 bmin, Vector2 bmax)
        {
            // Traverse tree
            List<RcChunkyTriMeshNode> ids = new();
            int i = 0;
            while (i < nodes.Count)
            {
                RcChunkyTriMeshNode node = nodes[i];
                bool overlap = CheckOverlapRect(bmin, bmax, node.bmin, node.bmax);
                bool isLeafNode = node.i >= 0;

                if (isLeafNode && overlap)
                {
                    ids.Add(node);
                }

                if (overlap || isLeafNode)
                {
                    i++;
                }
                else
                {
                    i = -node.i;
                }
            }

            return ids;
        }

        public List<RcChunkyTriMeshNode> GetChunksOverlappingSegment(float[] p, float[] q)
        {
            // Traverse tree
            List<RcChunkyTriMeshNode> ids = new();
            int i = 0;
            while (i < nodes.Count)
            {
                RcChunkyTriMeshNode node = nodes[i];
                bool overlap = CheckOverlapSegment(p, q, node.bmin, node.bmax);
                bool isLeafNode = node.i >= 0;

                if (isLeafNode && overlap)
                {
                    ids.Add(node);
                }

                if (overlap || isLeafNode)
                {
                    i++;
                }
                else
                {
                    i = -node.i;
                }
            }

            return ids;
        }

        private static bool CheckOverlapSegment(float[] p, float[] q, Vector2 bmin, Vector2 bmax)
        {
            float EPSILON = 1e-6f;

            float tmin = 0;
            float tmax = 1;
            float[] d = new float[2];
            d[0] = q[0] - p[0];
            d[1] = q[1] - p[1];

            for (int i = 0; i < 2; i++)
            {
                if (Math.Abs(d[i]) < EPSILON)
                {
                    // Ray is parallel to slab. No hit if origin not within slab
                    if (p[i] < bmin[i] || p[i] > bmax[i])
                        return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    float ood = 1f / d[i];
                    float t1 = (bmin[i] - p[i]) * ood;
                    float t2 = (bmax[i] - p[i]) * ood;
                    if (t1 > t2)
                    {
                        (t1, t2) = (t2, t1);
                    }

                    if (t1 > tmin)
                        tmin = t1;
                    if (t2 < tmax)
                        tmax = t2;
                    if (tmin > tmax)
                        return false;
                }
            }

            return true;
        }
    }
}