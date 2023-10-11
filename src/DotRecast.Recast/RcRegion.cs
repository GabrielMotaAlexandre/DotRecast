using System.Collections.Generic;

namespace DotRecast.Recast
{
    public struct RcRegion
    {
        public int spanCount; // Number of spans belonging to this region
        public int id; // ID of the region
        public int areaType; // Are type.
        public bool remap;
        public bool visited;
        public bool overlap;
        public bool connectsToBorder;
        public int ymin, ymax;
        public readonly List<int> connections;
        public readonly List<int> floors;

        public RcRegion(int i)
        {
            id = i;
            ymin = 0xFFFF;
            connections = new List<int>();
            floors = new List<int>();
        }
    }
}