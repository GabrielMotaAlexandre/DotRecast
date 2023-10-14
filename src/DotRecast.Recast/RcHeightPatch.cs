namespace DotRecast.Recast
{
    public  struct RcHeightPatch
    {
        public  int xmin;
        public  int ymin;
        public  int width;
        public  int height;
        public  int[] data;

        public RcHeightPatch(int xmin, int ymin, int width, int height, int[] data)
        {
            this.xmin = xmin;
            this.ymin = ymin;
            this.width = width;
            this.height = height;
            this.data = data;
        }
    }
}