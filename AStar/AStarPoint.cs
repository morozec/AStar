using System;
using System.Collections.Generic;

namespace AStar
{
    public abstract class AStarPoint : IComparable<AStarPoint>
    {
        public double G { get; set; }
        public double H { get; set; }

        public double F
        {
            get
            {
                return G + H;
            }
        }

        public AStarPoint CameFromPoint { get; set; }

        public abstract IEnumerable<AStarPoint> GetNeighbors();

        public abstract double GetHeuristicCost(AStarPoint goal);

        public abstract double GetCost(AStarPoint goal);

        public int CompareTo(AStarPoint other)
        {
            return -F.CompareTo(other.F);
        }

    }
}