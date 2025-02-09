﻿using System.Collections.Generic;

namespace DotRecast.Recast.Geom
{
    public class BoundsItemXComparer : IComparer<BoundsItem>
    {
        public static readonly BoundsItemXComparer Shared = new();

        private BoundsItemXComparer()
        {
        }

        public int Compare(BoundsItem a, BoundsItem b)
        {
            return a.bmin.X.CompareTo(b.bmin.X);
        }
    }
}