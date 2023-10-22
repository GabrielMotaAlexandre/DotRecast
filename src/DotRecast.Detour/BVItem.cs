using UnityEngine;

namespace DotRecast.Detour
{
    // todoperf
    public readonly struct BVItem
    {
        public readonly Vector3Int bmin;
        public readonly Vector3Int bmax;
        public readonly int i;

        public BVItem(Vector3Int bmin, Vector3Int bmax, int i)
        {
            this.bmin = bmin;
            this.bmax = bmax;
            this.i = i;
        }
    }
}