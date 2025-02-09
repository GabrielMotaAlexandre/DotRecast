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

namespace DotRecast.Detour.TileCache
{
    public class DtTileCacheObstacle
    {
        public readonly int index;
        public DtTileCacheObstacleType type;
        public Vector3 pos;
        public Vector3 bmin;
        public Vector3 bmax;
        public float radius, height;
        public Vector3 center;
        public Vector3 extents;
        public readonly float[] rotAux = new float[2]; // { Cos(0.5f*angle)*Sin(-0.5f*angle); Cos(0.5f*angle)*Cos(0.5f*angle) - 0.5 }
        public List<long> touched = new();
        public readonly List<long> pending = new();
        public int salt;
        public DtObstacleState state = DtObstacleState.DT_OBSTACLE_EMPTY;
        public DtTileCacheObstacle next;

        public DtTileCacheObstacle(int index)
        {
            salt = 1;
            this.index = index;
        }
    }
}