namespace DotRecast.Detour.Extras.Jumplink
{
    public class JumpLinkType
    {
        public const int EDGE_JUMP_BIT = 1 << 0;
        public const int EDGE_CLIMB_DOWN_BIT = 1 << 1;
        public const int EDGE_JUMP_OVER_BIT = 1 << 2;

        public static readonly JumpLinkType EDGE_JUMP = new(EDGE_JUMP_BIT);
        public static readonly JumpLinkType EDGE_CLIMB_DOWN = new(EDGE_CLIMB_DOWN_BIT);
        public static readonly JumpLinkType EDGE_JUMP_OVER = new(EDGE_JUMP_OVER_BIT);

        public int Bit { get; }

        private JumpLinkType(int bit)
        {
            Bit = bit;
        }
    }
}