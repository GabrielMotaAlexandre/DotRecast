﻿using DotRecast.Core;

namespace DotRecast.Recast.Geom
{
    public class ChunkyTriMeshNode
    {
        public RcVec2f bmin;
        public RcVec2f bmax;
        public int i;
        public int[] tris;
    }
}