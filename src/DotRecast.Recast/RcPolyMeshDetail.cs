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

using System.Collections.Generic;
using System.Numerics;
using UnityEngine;

namespace DotRecast.Recast
{
    /**
    * Contains triangle meshes that represent detailed height data associated with the polygons in its associated polygon
    * mesh object.
*/
    public readonly struct RcPolyMeshDetail
    {
        /** The sub-mesh data. [Size: 4*#nmeshes] */
        public readonly int[] meshes { get; init; }

        /** The mesh vertices. [Size: 3*#nverts] */
        public readonly Vector3[] verts { get; init; }

        /** The mesh triangles. [Size: 4*#ntris] */
        public readonly Vector4Int[] tris { get; init; }

        /** The number of sub-meshes defined by #meshes. */
        public readonly int nmeshes { get; init; }

        public readonly bool IsValid => meshes != null;
    }
}