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
using System.Numerics;
using DotRecast.Core;

namespace DotRecast.Recast
{
    public static class RcPolyMeshRaycast
    {
        public static bool Raycast(IList<RcBuilderResult> results, Vector3 src, Vector3 dst, out float hitTime)
        {
            hitTime = 0f;
            foreach (RcBuilderResult result in results)
            {
                if (result.MeshDetail.IsValid)
                {
                    if (Raycast(result.Mesh, result.MeshDetail, src, dst, out hitTime))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        private static bool Raycast(RcPolyMesh poly, RcPolyMeshDetail meshDetail, Vector3 sp, Vector3 sq, out float hitTime)
        {
            hitTime = 0;
            if (meshDetail.IsValid)
            {
                for (int i = 0; i < meshDetail.nmeshes; ++i)
                {
                    int m = i * 4;
                    int bverts = meshDetail.meshes[m];
                    int btris = meshDetail.meshes[m + 2];
                    int ntris = meshDetail.meshes[m + 3];
                    for (int j = 0; j < ntris; ++j)
                    {
                        if (Intersections.IntersectSegmentTriangle(sp, sq,
                            meshDetail.verts[bverts + meshDetail.tris[btris + j].x],
                            meshDetail.verts[bverts + meshDetail.tris[btris + j].y],
                            meshDetail.verts[bverts + meshDetail.tris[btris + j].z],
                            out hitTime))
                        {
                            return true;
                        }
                    }
                }
            }
            else
            {
                // TODO: check PolyMesh instead
                throw new NotSupportedException();
            }

            return false;
        }
    }
}