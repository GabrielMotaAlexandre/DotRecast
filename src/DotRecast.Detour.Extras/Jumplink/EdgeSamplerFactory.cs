using System;
using System.Numerics;

namespace DotRecast.Detour.Extras.Jumplink
{
    class EdgeSamplerFactory
    {
        public static EdgeSampler Get(JumpLinkBuilderConfig acfg, JumpLinkType type, JumpEdge edge)
        {
            EdgeSampler es = type.Bit switch
            {
                JumpLinkType.EDGE_JUMP_BIT => InitEdgeJumpSampler(acfg, edge),
                JumpLinkType.EDGE_CLIMB_DOWN_BIT => InitClimbDownSampler(acfg, edge),
                _ => throw new ArgumentException("Unsupported jump type " + type),
            };
            return es;
        }


        private static EdgeSampler InitEdgeJumpSampler(JumpLinkBuilderConfig acfg, JumpEdge edge)
        {
            EdgeSampler es = new(edge, new JumpTrajectory(acfg.jumpHeight));
            es.start.height = acfg.agentClimb * 2;
            Vector3 offset = new();
            Trans2d(ref offset, es.az, es.ay, new Vector2 { X = acfg.startDistance, Y = -acfg.agentClimb, });
            Vadd(ref es.start.p, edge.sp, offset);
            Vadd(ref es.start.q, edge.sq, offset);

            float dx = acfg.endDistance - 2 * acfg.agentRadius;
            float cs = acfg.cellSize;
            int nsamples = Math.Max(2, (int)MathF.Ceiling(dx / cs));

            for (int j = 0; j < nsamples; ++j)
            {
                float v = j / (float)(nsamples - 1);
                float ox = 2 * acfg.agentRadius + dx * v;
                Trans2d(ref offset, es.az, es.ay, new Vector2 { X = ox, Y = acfg.minHeight });
                GroundSegment end = new()
                {
                    height = acfg.heightRange
                };
                Vadd(ref end.p, edge.sp, offset);
                Vadd(ref end.q, edge.sq, offset);
                es.end.Add(end);
            }

            return es;
        }

        private static EdgeSampler InitClimbDownSampler(JumpLinkBuilderConfig acfg, JumpEdge edge)
        {
            EdgeSampler es = new(edge, new ClimbTrajectory());
            es.start.height = acfg.agentClimb * 2;
            Vector3 offset = new();
            Trans2d(ref offset, es.az, es.ay, new Vector2() { X = acfg.startDistance, Y = -acfg.agentClimb });
            Vadd(ref es.start.p, edge.sp, offset);
            Vadd(ref es.start.q, edge.sq, offset);

            Trans2d(ref offset, es.az, es.ay, new Vector2() { X = acfg.endDistance, Y = acfg.minHeight });
            GroundSegment end = new()
            {
                height = acfg.heightRange
            };
            Vadd(ref end.p, edge.sp, offset);
            Vadd(ref end.q, edge.sq, offset);
            es.end.Add(end);
            return es;
        }

        private static void Vadd(ref Vector3 dest, Vector3 v1, Vector3 v2)
        {
            dest.X = v1.X + v2.X;
            dest.Y = v1.Y + v2.Y;
            dest.Z = v1.Z + v2.Z;
        }

        private static void Trans2d(ref Vector3 dst, Vector3 ax, Vector3 ay, Vector2 pt)
        {
            dst.X = ax.X * pt.X + ay.X * pt.Y;
            dst.Y = ax.Y * pt.X + ay.Y * pt.Y;
            dst.Z = ax.Z * pt.X + ay.Z * pt.Y;
        }
    }
}