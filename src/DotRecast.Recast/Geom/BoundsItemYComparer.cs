using System.Collections.Generic;

namespace DotRecast.Recast.Geom
{
    public class BoundsItemYComparer : IComparer<BoundsItem>
    {
        public static readonly BoundsItemYComparer Shared = new();

        private BoundsItemYComparer()
        {
        }

        public int Compare(BoundsItem a, BoundsItem b)
        {
            return a.bmin.y.CompareTo(b.bmin.y);
        }
    }
}