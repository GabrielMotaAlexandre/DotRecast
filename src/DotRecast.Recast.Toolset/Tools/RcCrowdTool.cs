using System;
using System.Collections.Generic;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Detour.Crowd;
using DotRecast.Detour.Crowd.Tracking;
using DotRecast.Recast.Toolset.Builder;

namespace DotRecast.Recast.Toolset.Tools
{
    public class RcCrowdTool : IRcToolable
    {
        private readonly DtCrowdAgentConfig _agCfg;
        private DtCrowd crowd;
        private readonly DtCrowdAgentDebugInfo _agentDebug;
        private long crowdUpdateTime;
        private readonly Dictionary<long, RcCrowdAgentTrail> _trails;
        private long _moveTargetRef;
        private Vector3 _moveTargetPos;

        public RcCrowdTool()
        {
            _agCfg = new DtCrowdAgentConfig();
            _agentDebug = new DtCrowdAgentDebugInfo
            {
                vod = new DtObstacleAvoidanceDebugData(2048)
            };
            _trails = new Dictionary<long, RcCrowdAgentTrail>();
        }


        public string GetName()
        {
            return "Crowd Control";
        }

        public DtCrowdAgentConfig GetCrowdConfig()
        {
            return _agCfg;
        }

        public DtCrowdAgentDebugInfo GetCrowdAgentDebugInfo()
        {
            return _agentDebug;
        }

        public Dictionary<long, RcCrowdAgentTrail> GetCrowdAgentTrails()
        {
            return _trails;
        }

        public long GetMoveTargetRef()
        {
            return _moveTargetRef;
        }

        public Vector3 GetMoveTargetPos()
        {
            return _moveTargetPos;
        }

        public void Setup(float agentRadius, DtNavMesh navMesh)
        {
            DtCrowdConfig config = new(agentRadius);
            crowd = new DtCrowd(config, navMesh, __ => new DtQueryDefaultFilter(
                SampleAreaModifications.SAMPLE_POLYFLAGS_ALL,
                SampleAreaModifications.SAMPLE_POLYFLAGS_DISABLED,
                new float[] { 1f, 10f, 1f, 1f, 2f, 1.5f })
            );

            // Setup local avoidance option to different qualities.
            // Use mostly default settings, copy from dtCrowd.
            DtObstacleAvoidanceParams option = new(crowd.GetObstacleAvoidanceParams(0))
            {
                // Low (11)
                velBias = 0.5f,
                adaptiveDivs = 5,
                adaptiveRings = 2,
                adaptiveDepth = 1
            };
            crowd.SetObstacleAvoidanceParams(0, option);

            // Medium (22)
            option.velBias = 0.5f;
            option.adaptiveDivs = 5;
            option.adaptiveRings = 2;
            option.adaptiveDepth = 2;
            crowd.SetObstacleAvoidanceParams(1, option);

            // Good (45)
            option.velBias = 0.5f;
            option.adaptiveDivs = 7;
            option.adaptiveRings = 2;
            option.adaptiveDepth = 3;
            crowd.SetObstacleAvoidanceParams(2, option);

            // High (66)
            option.velBias = 0.5f;
            option.adaptiveDivs = 7;
            option.adaptiveRings = 3;
            option.adaptiveDepth = 3;

            crowd.SetObstacleAvoidanceParams(3, option);
        }

        public void UpdateAgentParams()
        {
            if (crowd is null)
            {
                return;
            }

            foreach (DtCrowdAgent ag in crowd.GetActiveAgents())
            {
                DtCrowdAgentParams agOption = new()
                {
                    radius = ag.option.radius,
                    height = ag.option.height,
                    maxAcceleration = ag.option.maxAcceleration,
                    maxSpeed = ag.option.maxSpeed,
                    collisionQueryRange = ag.option.collisionQueryRange,
                    pathOptimizationRange = ag.option.pathOptimizationRange,
                    obstacleAvoidanceType = ag.option.obstacleAvoidanceType,
                    queryFilterType = ag.option.queryFilterType,
                    userData = ag.option.userData,
                    updateFlags = _agCfg.GetUpdateFlags()
                };
                agOption.obstacleAvoidanceType = _agCfg.obstacleAvoidanceType;
                agOption.separationWeight = _agCfg.separationWeight;
                DtCrowd.UpdateAgentParameters(ag, agOption);
            }
        }

        public DtCrowd GetCrowd()
        {
            return crowd;
        }

        public void Update(float dt)
        {
            if (crowd is null)
                return;

            DtNavMesh nav = crowd.GetNavMesh();
            if (nav is null)
                return;

            long startTime = RcFrequency.Ticks;
            crowd.Update(dt, _agentDebug);
            long endTime = RcFrequency.Ticks;

            // Update agent trails
            foreach (DtCrowdAgent ag in crowd.GetActiveAgents())
            {
                RcCrowdAgentTrail trail = _trails[ag.idx];
                // Update agent movement trail.
                trail.htrail = (trail.htrail + 1) % RcCrowdAgentTrail.AGENT_MAX_TRAIL;
                trail.trail[trail.htrail * 3] = ag.npos.X;
                trail.trail[trail.htrail * 3 + 1] = ag.npos.Y;
                trail.trail[trail.htrail * 3 + 2] = ag.npos.Z;
            }

            _agentDebug.vod.NormalizeSamples();

            // m_crowdSampleCount.addSample((float) crowd.GetVelocitySampleCount());
            crowdUpdateTime = (endTime - startTime) / TimeSpan.TicksPerMillisecond;
        }

        public void RemoveAgent(DtCrowdAgent agent)
        {
            crowd.RemoveAgent(agent);
            if (agent == _agentDebug.agent)
            {
                _agentDebug.agent = null;
            }
        }

        public void AddAgent(Vector3 p, float agentRadius, float agentHeight, float agentMaxAcceleration, float agentMaxSpeed)
        {
            DtCrowdAgentParams ap = CreateAgentParams(agentRadius, agentHeight, agentMaxAcceleration, agentMaxSpeed);
            DtCrowdAgent ag = crowd.AddAgent(p, ap);
            if (ag != null)
            {
                if (_moveTargetRef != 0)
                    DtCrowd.RequestMoveTarget(ag, _moveTargetRef, _moveTargetPos);

                // Init trail
                if (!_trails.TryGetValue(ag.idx, out var trail))
                {
                    trail = new RcCrowdAgentTrail();
                    _trails.Add(ag.idx, trail);
                }

                for (int i = 0; i < RcCrowdAgentTrail.AGENT_MAX_TRAIL; ++i)
                {
                    trail.trail[i * 3] = p.X;
                    trail.trail[i * 3 + 1] = p.Y;
                    trail.trail[i * 3 + 2] = p.Z;
                }

                trail.htrail = 0;
            }
        }

        private DtCrowdAgentParams CreateAgentParams(float agentRadius, float agentHeight, float agentMaxAcceleration, float agentMaxSpeed)
        {
            DtCrowdAgentParams ap = new()
            {
                radius = agentRadius,
                height = agentHeight,
                maxAcceleration = agentMaxAcceleration,
                maxSpeed = agentMaxSpeed
            };
            ap.collisionQueryRange = ap.radius * 12f;
            ap.pathOptimizationRange = ap.radius * 30f;
            ap.updateFlags = _agCfg.GetUpdateFlags();
            ap.obstacleAvoidanceType = _agCfg.obstacleAvoidanceType;
            ap.separationWeight = _agCfg.separationWeight;
            return ap;
        }

        public DtCrowdAgent HitTestAgents(Vector3 s, Vector3 p)
        {
            DtCrowdAgent isel = null;
            float tsel = float.MaxValue;

            foreach (DtCrowdAgent ag in crowd.GetActiveAgents())
            {
                Vector3 bmin = new();
                Vector3 bmax = new();
                GetAgentBounds(ag, ref bmin, ref bmax);
                if (Intersections.IsectSegAABB(s, p, bmin, bmax, out var tmin, out var tmax))
                {
                    if (tmin > 0 && tmin < tsel)
                    {
                        isel = ag;
                        tsel = tmin;
                    }
                }
            }

            return isel;
        }

        private static void GetAgentBounds(DtCrowdAgent ag, ref Vector3 bmin, ref Vector3 bmax)
        {
            Vector3 p = ag.npos;
            float r = ag.option.radius;
            float h = ag.option.height;
            bmin.X = p.X - r;
            bmin.Y = p.Y;
            bmin.Z = p.Z - r;
            bmax.X = p.X + r;
            bmax.Y = p.Y + h;
            bmax.Z = p.Z + r;
        }

        public void SetMoveTarget(Vector3 p, bool adjust)
        {
            if (crowd is null)
                return;

            // Find nearest point on navmesh and set move request to that location.
            DtNavMeshQuery navquery = crowd.GetNavMeshQuery();
            IDtQueryFilter filter = crowd.GetFilter(0);
            Vector3 halfExtents = crowd.GetQueryExtents();

            if (adjust)
            {
                // Request velocity
                if (_agentDebug.agent != null)
                {
                    Vector3 vel = CalcVel(_agentDebug.agent.npos, p, _agentDebug.agent.option.maxSpeed);
                    DtCrowd.RequestMoveVelocity(_agentDebug.agent, vel);
                }
                else
                {
                    foreach (DtCrowdAgent ag in crowd.GetActiveAgents())
                    {
                        Vector3 vel = CalcVel(ag.npos, p, ag.option.maxSpeed);
                        DtCrowd.RequestMoveVelocity(ag, vel);
                    }
                }
            }
            else
            {
                navquery.FindNearestPoly(p, halfExtents, filter, out _moveTargetRef, out _moveTargetPos, out var _);
                if (_agentDebug.agent != null)
                {
                    DtCrowd.RequestMoveTarget(_agentDebug.agent, _moveTargetRef, _moveTargetPos);
                }
                else
                {
                    foreach (DtCrowdAgent ag in crowd.GetActiveAgents())
                    {
                        DtCrowd.RequestMoveTarget(ag, _moveTargetRef, _moveTargetPos);
                    }
                }
            }
        }

        private static Vector3 CalcVel(Vector3 pos, Vector3 tgt, float speed)
        {
            Vector3 vel = tgt - pos;
            vel.Y = 0f;
            vel.Normalize();
            return vel * speed;
        }

        public long GetCrowdUpdateTime()
        {
            return crowdUpdateTime;
        }

        public void HighlightAgent(DtCrowdAgent agent)
        {
            _agentDebug.agent = agent;
        }
    }
}