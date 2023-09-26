using DotRecast.Core;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcDynamicUpdateToolMode
    {
        public static readonly RcDynamicUpdateToolMode BUILD = new(0, "Build");
        public static readonly RcDynamicUpdateToolMode COLLIDERS = new(1, "Colliders");
        public static readonly RcDynamicUpdateToolMode RAYCAST = new(2, "Raycast");

        public static readonly RcImmutableArray<RcDynamicUpdateToolMode> Values = RcImmutableArray.Create(
            BUILD, COLLIDERS, RAYCAST
        );

        public int Idx { get; }
        public string Label { get; }

        private RcDynamicUpdateToolMode(int idx, string label)
        {
            Idx = idx;
            Label = label;
        }
    }
}