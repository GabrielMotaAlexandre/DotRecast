using System;
using System.Numerics;

namespace DotRecast.Detour
{
    public class DtNoOpDtPolygonByCircleConstraint : IDtPolygonByCircleConstraint
    {
        public static readonly DtNoOpDtPolygonByCircleConstraint Shared = new();

        private DtNoOpDtPolygonByCircleConstraint()
        {
        }

        public ReadOnlySpan<float> Apply(ReadOnlySpan<float> polyVerts, Vector3 circleCenter, float radius)
        {
            return polyVerts;
        }
    }
}