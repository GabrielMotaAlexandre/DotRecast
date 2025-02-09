namespace DotRecast.Detour.Crowd
{
    public enum DtMoveRequestState
    {
        DT_CROWDAGENT_TARGET_NONE,
        // todo: should we use?
        DT_CROWDAGENT_TARGET_STOPPED,
        DT_CROWDAGENT_TARGET_FAILED,
        DT_CROWDAGENT_TARGET_VALID,
        DT_CROWDAGENT_TARGET_REQUESTING,
        // The path is longer or potentially unreachable, full plan.
        DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
        DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
        DT_CROWDAGENT_TARGET_VELOCITY,
    };
}