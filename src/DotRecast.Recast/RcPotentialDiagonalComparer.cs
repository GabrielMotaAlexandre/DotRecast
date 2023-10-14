using System.Collections.Generic;

namespace DotRecast.Recast
{
    public struct RcPotentialDiagonalComparer : IComparer<RcPotentialDiagonal>
    {
        public static readonly RcPotentialDiagonalComparer Shared = new();

        public readonly int Compare(RcPotentialDiagonal va, RcPotentialDiagonal vb)
        {
            RcPotentialDiagonal a = va;
            RcPotentialDiagonal b = vb;
            return a.dist.CompareTo(b.dist);
        }
    }
}