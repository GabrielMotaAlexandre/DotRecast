﻿using DotRecast.Core;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcTestNavmeshToolMode
    {
        public static readonly RcTestNavmeshToolMode PATHFIND_FOLLOW = new(0, "Pathfind Follow");
        public static readonly RcTestNavmeshToolMode PATHFIND_STRAIGHT = new(1, "Pathfind Straight");
        public static readonly RcTestNavmeshToolMode PATHFIND_SLICED = new(2, "Pathfind Sliced");
        public static readonly RcTestNavmeshToolMode DISTANCE_TO_WALL = new(3, "Distance to Wall");
        public static readonly RcTestNavmeshToolMode RAYCAST = new(4, "Raycast");
        public static readonly RcTestNavmeshToolMode FIND_POLYS_IN_CIRCLE = new(5, "Find Polys in Circle");
        public static readonly RcTestNavmeshToolMode FIND_POLYS_IN_SHAPE = new(6, "Find Polys in Shape");
        public static readonly RcTestNavmeshToolMode FIND_LOCAL_NEIGHBOURHOOD = new(7, "Find Local Neighbourhood");
        public static readonly RcTestNavmeshToolMode RANDOM_POINTS_IN_CIRCLE = new(8, "Random Points in Circle");

        public static readonly RcImmutableArray<RcTestNavmeshToolMode> Values = RcImmutableArray.Create(
            PATHFIND_FOLLOW,
            PATHFIND_STRAIGHT,
            PATHFIND_SLICED,
            DISTANCE_TO_WALL,
            RAYCAST,
            FIND_POLYS_IN_CIRCLE,
            FIND_POLYS_IN_SHAPE,
            FIND_LOCAL_NEIGHBOURHOOD,
            RANDOM_POINTS_IN_CIRCLE
        );


        public int Idx { get; }
        public string Label { get; }

        private RcTestNavmeshToolMode(int idx, string label)
        {
            Idx = idx;
            Label = label;
        }
    }
}