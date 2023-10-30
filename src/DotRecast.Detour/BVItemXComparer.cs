using System.Collections.Generic;

namespace DotRecast.Detour
{
    public readonly struct BVItemXComparer : IComparer<BVItem>
    {
        public static readonly BVItemXComparer Shared = new();

        public readonly int Compare(BVItem a, BVItem b)
        {
            return a.bmin[0].CompareTo(b.bmin[0]);
        }
    }
}