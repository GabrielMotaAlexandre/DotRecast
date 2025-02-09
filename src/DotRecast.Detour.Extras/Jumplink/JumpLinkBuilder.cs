using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Recast;

namespace DotRecast.Detour.Extras.Jumplink
{
    public class JumpLinkBuilder
    {
        private readonly NavMeshGroundSampler groundSampler = new();

        private readonly List<JumpEdge[]> edges;
        private readonly IList<RcBuilderResult> results;

        public JumpLinkBuilder(IList<RcBuilderResult> results)
        {
            this.results = results;
            edges = results.Select(r => EdgeExtractor.ExtractEdges(r.Mesh)).ToList();
        }

        public List<JumpLink> Build(JumpLinkBuilderConfig acfg, JumpLinkType type)
        {
            List<JumpLink> links = new();
            for (int tile = 0; tile < results.Count; tile++)
            {
                JumpEdge[] edges = this.edges[tile];
                foreach (JumpEdge edge in edges)
                {
                    links.AddRange(ProcessEdge(acfg, results[tile], type, edge));
                }
            }

            return links;
        }

        private List<JumpLink> ProcessEdge(JumpLinkBuilderConfig acfg, RcBuilderResult result, JumpLinkType type, JumpEdge edge)
        {
            EdgeSampler es = EdgeSamplerFactory.Get(acfg, type, edge);
            groundSampler.Sample(acfg, result, es);
            TrajectorySampler.Sample(acfg, in result.Solid, es);
            JumpSegment[] jumpSegments = JumpSegmentBuilder.Build(acfg, es);
            return BuildJumpLinks(acfg, es, jumpSegments);
        }


        private static List<JumpLink> BuildJumpLinks(JumpLinkBuilderConfig acfg, EdgeSampler es, JumpSegment[] jumpSegments)
        {
            List<JumpLink> links = new();
            foreach (JumpSegment js in jumpSegments)
            {
                Vector3 sp = es.start.gsamples[js.startSample].p;
                Vector3 sq = es.start.gsamples[js.startSample + js.samples - 1].p;
                GroundSegment end = es.end[js.groundSegment];
                Vector3 ep = end.gsamples[js.startSample].p;
                Vector3 eq = end.gsamples[js.startSample + js.samples - 1].p;
                float d = Math.Min(Vector3Extensions.Dist2DSqr(sp, sq), Vector3Extensions.Dist2DSqr(ep, eq));
                if (d >= 4 * acfg.agentRadius * acfg.agentRadius)
                {
                    JumpLink link = new();
                    links.Add(link);
                    link.startSamples = RcArrayUtils.CopyOf(es.start.gsamples, js.startSample, js.samples);
                    link.endSamples = RcArrayUtils.CopyOf(end.gsamples, js.startSample, js.samples);
                    link.start = es.start;
                    link.end = end;
                    link.trajectory = es.trajectory;
                    for (int j = 0; j < link.nspine; ++j)
                    {
                        float u = ((float)j) / (link.nspine - 1);
                        Vector3 p = es.trajectory.Apply(sp, ep, u);
                        link.spine0[j * 3] = p.X;
                        link.spine0[j * 3 + 1] = p.Y;
                        link.spine0[j * 3 + 2] = p.Z;

                        p = es.trajectory.Apply(sq, eq, u);
                        link.spine1[j * 3] = p.X;
                        link.spine1[j * 3 + 1] = p.Y;
                        link.spine1[j * 3 + 2] = p.Z;
                    }
                }
            }

            return links;
        }

        public List<JumpEdge[]> GetEdges()
        {
            return edges;
        }
    }
}