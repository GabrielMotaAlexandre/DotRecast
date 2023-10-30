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

        public ReadOnlySpan<Vector3> Apply(ReadOnlySpan<Vector3> polyVerts, Vector3 circleCenter, float radius)
        {
            return polyVerts;
        }
    }
}