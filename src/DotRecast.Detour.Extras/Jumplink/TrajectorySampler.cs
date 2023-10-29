using System;
using System.Numerics;
using DotRecast.Recast;


namespace DotRecast.Detour.Extras.Jumplink
{
    class TrajectorySampler
    {
        public static void Sample(JumpLinkBuilderConfig acfg, in RcHeightfield heightfield, EdgeSampler es)
        {
            int nsamples = es.start.gsamples.Length;
            for (int i = 0; i < nsamples; ++i)
            {
                GroundSample ssmp = es.start.gsamples[i];
                foreach (GroundSegment end in es.end)
                {
                    GroundSample esmp = end.gsamples[i];
                    if (!ssmp.validHeight || !esmp.validHeight)
                    {
                        continue;
                    }

                    if (!SampleTrajectory(acfg, in heightfield, ssmp.p, esmp.p, es.trajectory))
                    {
                        continue;
                    }

                    ssmp.validTrajectory = true;
                    esmp.validTrajectory = true;
                }
            }
        }

        private static bool SampleTrajectory(JumpLinkBuilderConfig acfg, in RcHeightfield solid, Vector3 pa, Vector3 pb, Trajectory tra)
        {
            float cs = Math.Min(acfg.cellSize, acfg.cellHeight);
            float d = Vector3Extensions.Dist2D(pa, pb) + Math.Abs(pa.Y - pb.Y);
            int nsamples = Math.Max(2, (int)MathF.Ceiling(d / cs));
            for (int i = 0; i < nsamples; ++i)
            {
                float u = i / (float)(nsamples - 1);
                Vector3 p = tra.Apply(pa, pb, u);
                if (CheckHeightfieldCollision(solid, p.X, p.Y + acfg.groundTolerance, p.Y + acfg.agentHeight, p.Z))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool CheckHeightfieldCollision(in RcHeightfield solid, float x, float ymin, float ymax, float z)
        {
            int w = solid.width;
            int h = solid.height;
            float cs = solid.cs;
            float ch = solid.ch;
            Vector3 orig = solid.bmin;
            int ix = (int)MathF.Floor((x - orig.X) / cs);
            int iz = (int)MathF.Floor((z - orig.Z) / cs);

            if (ix < 0 || iz < 0 || ix > w || iz > h)
            {
                return false;
            }

            var list = solid.spans[ix + iz * w];
            if (list is null)
            {
                return false;
            }

            for (int i = 0; i < list.Count; i++)
            {
                var span = list[i];
                float symin = orig.Y + span.smin * ch;
                float symax = orig.Y + span.smax * ch;
                if (OverlapRange(ymin, ymax, symin, symax))
                {
                    return true;
                }
            }

            return false;
        }

        private static bool OverlapRange(float amin, float amax, float bmin, float bmax)
        {
            return amin <= bmax && amax >= bmin;
        }
    }
}