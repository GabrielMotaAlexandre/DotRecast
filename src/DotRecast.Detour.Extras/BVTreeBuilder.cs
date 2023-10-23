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
using System.Numerics;
using UnityEngine;

namespace DotRecast.Detour.Extras
{
    public class BVTreeBuilder
    {
        public static void Build(DtMeshData data)
        {
            data.bvTree = new DtBVNode[data.header.polyCount * 2];
            data.header.bvNodeCount = data.bvTree.Length is 0
                ? 0
                : CreateBVTree(data, data.bvTree, data.header.bvQuantFactor);
        }

        private static int CreateBVTree(DtMeshData data, Span<DtBVNode> nodes, float quantFactor)
        {
            Span<int> bmin = stackalloc int[3];
            Span<int> bmax = stackalloc int[3];

            var items = new BVItem[data.header.polyCount];
            for (int i = 0; i < data.header.polyCount; i++)
            {
                var bminT = data.verts.UnsafeAs<float, Vector3>(data.polys[i].verts[0]);
                var bmaxT = bminT;
                for (int j = 1; j < data.polys[i].vertCount; j++)
                {
                    bminT.Min(data.verts, data.polys[i].verts[j] * 3);
                    bmaxT.Max(data.verts, data.polys[i].verts[j] * 3);
                }

                bmin[0] = Math.Clamp((int)((bminT.X - data.header.bmin.X) * quantFactor), 0, 0x7fffffff);
                bmin[1] = Math.Clamp((int)((bminT.Y - data.header.bmin.Y) * quantFactor), 0, 0x7fffffff);
                bmin[2] = Math.Clamp((int)((bminT.Z - data.header.bmin.Z) * quantFactor), 0, 0x7fffffff);
                bmax[0] = Math.Clamp((int)((bmaxT.X - data.header.bmin.X) * quantFactor), 0, 0x7fffffff);
                bmax[1] = Math.Clamp((int)((bmaxT.Y - data.header.bmin.Y) * quantFactor), 0, 0x7fffffff);
                bmax[2] = Math.Clamp((int)((bmaxT.Z - data.header.bmin.Z) * quantFactor), 0, 0x7fffffff);


                items[i] = new BVItem(bmin.UnsafeAs<int, Vector3Int>(), bmax.UnsafeAs<int, Vector3Int>(), i);
            }

            return DtNavMeshBuilder.Subdivide(items, data.header.polyCount, 0, data.header.polyCount, 0, nodes);
        }
    }
}